#include <easy3d/algo/delaunay_3d.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/model.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/kdtree/kdtree_search_eth.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/initializer.h>
#include <easy3d/util/resource.h>
#include <easy3d/viewer/viewer.h>

#include <filesystem>
#include <iostream>
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include "custom_3d_regularization.h"
#include "spherical_kMeans.h"

using namespace easy3d;
using namespace rerun::demo;

std::vector<Drawable*> drawables;  // store drawables added to the viewer

// function declarations
bool run_easy3d_kdTree_graph_approach(Viewer* viewer, Model* model);
bool offset_xyz(Viewer* viewer, Model* model);
Graph* build_knn_graph(PointCloud* cloud, int k);
Graph* build_delaunay_graph(PointCloud* cloud);
Graph* combine_graphs(Graph* knn_graph, Graph* delaunay_graph, float max_edge_length);

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path>" << std::endl;
        return -1;
    }

    std::string input_file_path = argv[1];

    initialize(true);
    LOG(INFO) << "Easy3D initialized";

    Viewer viewer("Geomatics Thesis");
    Model* model = viewer.add_model(input_file_path, true);
    offset_xyz(&viewer, model);

    // set up rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_impostor_type(PointsDrawable::PLAIN);
    drawable->set_point_size(3.0f);

    // set usage instructions
    viewer.set_usage("'Ctrl + k': run kdTree graph approach\n");

    // bind function to key
    viewer.bind(run_easy3d_kdTree_graph_approach, model, Viewer::KEY_K, Viewer::MODIF_CTRL);

    // fit screen
    viewer.fit_screen();

    return viewer.run();
}

bool offset_xyz(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto points = cloud->get_vertex_property<vec3>("v:point");
    float min_x = INFINITY;
    float min_y = INFINITY;
    float min_z = INFINITY;

    for (auto vertex : cloud->vertices()) {
        const vec3& point = points[vertex];
        if (point.x < min_x) min_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.z < min_z) min_z = point.z;
    }

    LOG(INFO) << "Offsetting xyz by " << min_x << ", " << min_y << ", " << min_z;
    LOG(INFO) << "Original point 0: " << points[PointCloud::Vertex(0)];

    for (auto vertex : cloud->vertices()) {
        points[vertex].x -= min_x;
        points[vertex].y -= min_y;
        points[vertex].z -= min_z;
    }

    LOG(INFO) << "Offset point 0:" << points[PointCloud::Vertex(0)];
    cloud->add_vertex_property<vec3>("v:offset_vector", vec3(min_x, min_y, min_z));
    return true;
}

// build k-nearest neighbors graph
Graph* build_knn_graph(PointCloud* cloud, int k) {
    // construct kdtree for the point cloud
    PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");
    auto tree = KdTreeSearch_ETH(cloud);
    Graph* graph = new Graph;
    std::map<int, Graph::Vertex> vertex_map;
    std::set<std::pair<int, int>> added_edges;

    // first pass: add all points as vertices
    for (const auto& v : cloud->vertices()) {
        vec3 p = points[v];
        Graph::Vertex gv = graph->add_vertex(p);
        vertex_map[v.idx()] = gv;
    }

    // second pass: add edges based on k-nearest neighbors
    for (const auto& v : cloud->vertices()) {
        vec3 p = points[v];
        std::vector<int> neighbors_indices;
        std::vector<float> neighbors_squared_distances;

        tree.find_closest_k_points(p, k, neighbors_indices, neighbors_squared_distances);

        // add edges between current point and its neighbors
        for (const auto& neighbor_idx : neighbors_indices) {
            if (neighbor_idx != v.idx()) {  // avoid self-loops
                // ensure we add each edge only once by ordering vertex indices
                int min_idx = std::min(v.idx(), neighbor_idx);
                int max_idx = std::max(v.idx(), neighbor_idx);
                auto edge_pair = std::make_pair(min_idx, max_idx);

                if (added_edges.find(edge_pair) == added_edges.end()) {
                    graph->add_edge(vertex_map[min_idx], vertex_map[max_idx]);
                    added_edges.insert(edge_pair);
                }
            }
        }
    }

    LOG(INFO) << "KNN graph constructed with " << graph->vertices_size() << " vertices and "
              << graph->edges_size() << " edges";
    return graph;
}

// build delaunay triangulation graph
Graph* build_delaunay_graph(PointCloud* cloud) {
    LOG(INFO) << "Building Delaunay triangulation graph...";
    Graph* graph = new Graph;
    std::map<int, Graph::Vertex> vertex_map;
    PointCloud::VertexProperty<vec3> points = cloud->get_vertex_property<vec3>("v:point");

    // add vertices
    for (const auto& v : cloud->vertices()) {
        vec3 p = points[v];
        Graph::Vertex gv = graph->add_vertex(p);
        vertex_map[v.idx()] = gv;
    }

    // build delaunay triangulation
    std::vector<vec3> point_array;
    for (const auto& v : cloud->vertices()) {
        point_array.push_back(points[v]);
    }
    Delaunay3 delaunay;
    delaunay.set_vertices(point_array);

    // extract edges from delaunay triangulation
    std::set<std::pair<int, int>> edges;
    for (unsigned int i = 0; i < delaunay.nb_tets(); ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = j + 1; k < 4; ++k) {
                int v1 = delaunay.tet_vertex(i, j);
                int v2 = delaunay.tet_vertex(i, k);
                int min_idx = std::min(v1, v2);
                int max_idx = std::max(v1, v2);
                edges.insert(std::make_pair(min_idx, max_idx));
            }
        }
    }

    // add edges to graph
    for (const auto& edge : edges) {
        graph->add_edge(vertex_map[edge.first], vertex_map[edge.second]);
    }

    LOG(INFO) << "Delaunay graph construction completed, containing " << graph->edges_size()
              << " edges";
    return graph;
}

// combine two graphs with edge length filtering
Graph* combine_graphs(Graph* knn_graph, Graph* delaunay_graph, float max_edge_length) {
    Graph* combined_graph = new Graph(*knn_graph);  // copy knn graph as base
    std::set<std::pair<int, int>> existing_edges;

    // collect existing edges
    for (const auto& e : knn_graph->edges()) {
        auto source = knn_graph->source(e);
        auto target = knn_graph->target(e);
        int min_idx = std::min(source.idx(), target.idx());
        int max_idx = std::max(source.idx(), target.idx());
        existing_edges.insert(std::make_pair(min_idx, max_idx));
    }

    // add filtered delaunay edges
    size_t new_edges_count = 0;
    for (const auto& e : delaunay_graph->edges()) {
        auto source = delaunay_graph->source(e);
        auto target = delaunay_graph->target(e);
        auto start = delaunay_graph->position(source);
        auto end = delaunay_graph->position(target);

        // check edge length
        float edge_length = (end - start).length();
        if (edge_length > max_edge_length) {
            continue;
        }

        int min_idx = std::min(source.idx(), target.idx());
        int max_idx = std::max(source.idx(), target.idx());
        auto edge_pair = std::make_pair(min_idx, max_idx);

        // add edge if not exists
        if (existing_edges.find(edge_pair) == existing_edges.end()) {
            combined_graph->add_edge(source, target);
            existing_edges.insert(edge_pair);
            ++new_edges_count;
        }
    }

    LOG(INFO) << "Added " << new_edges_count << " new edges (length <= " << max_edge_length
              << "m) from Delaunay graph";
    return combined_graph;
}

bool run_easy3d_kdTree_graph_approach(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    // create rerun logger
    const auto rr = rerun::RecordingStream("kdTree Approach logger");
    rr.spawn().exit_on_failure();

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto points = cloud->get_vertex_property<vec3>("v:point");

    // log point cloud to rerun
    std::vector<rerun::Position3D> rr_point_cloud;
    for (const auto& v : cloud->vertices()) {
        vec3 p = points[v];
        rr_point_cloud.push_back(rerun::Position3D{p.x, p.y, p.z});
    }
    rr.log("points", rerun::Points3D(rr_point_cloud).with_radii({0.05f}));

    // build knn graph
    int k_neighbors = 16;
    Graph* knn_graph = build_knn_graph(cloud, k_neighbors);

    // log original knn graph
    std::vector<rerun::Collection<rerun::Vec3D>> strips3d;
    for (const auto& e : knn_graph->edges()) {
        auto source = knn_graph->source(e);
        auto target = knn_graph->target(e);
        auto start = knn_graph->position(source);
        auto end = knn_graph->position(target);
        rerun::Collection<rerun::Vec3D> strip = {
            {static_cast<float>(start.x), static_cast<float>(start.y), static_cast<float>(start.z)},
            {static_cast<float>(end.x), static_cast<float>(end.y), static_cast<float>(end.z)}};
        strips3d.push_back(strip);
    }
    rr.log("original_graph", rerun::LineStrips3D(strips3d).with_radii({0.01f}));

    // build delaunay graph
    Graph* delaunay_graph = build_delaunay_graph(cloud);

    // combine graphs
    const float max_edge_length = 2.0f;
    Graph* global_graph = combine_graphs(knn_graph, delaunay_graph, max_edge_length);

    // log combined graph
    std::vector<rerun::Collection<rerun::Vec3D>> combined_strips3d;
    for (const auto& e : global_graph->edges()) {
        auto source = global_graph->source(e);
        auto target = global_graph->target(e);
        auto start = global_graph->position(source);
        auto end = global_graph->position(target);
        rerun::Collection<rerun::Vec3D> strip = {
            {static_cast<float>(start.x), static_cast<float>(start.y), static_cast<float>(start.z)},
            {static_cast<float>(end.x), static_cast<float>(end.y), static_cast<float>(end.z)}};
        combined_strips3d.push_back(strip);
    }
    rr.log("combined_graph", rerun::LineStrips3D(combined_strips3d).with_radii({0.01f}));

    // transform to CGAL segments
    std::vector<custom_3d::Segment_3> segments;
    for (const auto& e : global_graph->edges()) {
        vec3 source = global_graph->position(global_graph->source(e));
        vec3 target = global_graph->position(global_graph->target(e));
        custom_3d::Point_3 s(source.x, source.y, source.z);
        custom_3d::Point_3 t(target.x, target.y, target.z);
        custom_3d::Segment_3 seg(s, t);
        segments.push_back(seg);
    }

    // execute global 3D QP regularization
    // custom_3d::Combined_regularization_3::Parameters params(60, 0.5, 15.0, 0.5);
    // custom_3d::Combined_regularization_3::regularize(segments, params);
    // custom_3d::Angle_regularization_3::regularize_with_batches(segments, 45);
    custom_3d::Offset_regularization_3::regularize_with_batches(segments, 0.1);

    // log regularized segments
    std::vector<rerun::Collection<rerun::Vec3D>> qp_strips3d;
    for (const auto& seg : segments) {
        auto s = seg.source;
        auto t = seg.target;
        rerun::Collection<rerun::Vec3D> strip = {
            {static_cast<float>(s.x()), static_cast<float>(s.y()), static_cast<float>(s.z())},
            {static_cast<float>(t.x()), static_cast<float>(t.y()), static_cast<float>(t.z())}};
        qp_strips3d.push_back(strip);
    }
    rr.log("regularized_segments", rerun::LineStrips3D(qp_strips3d).with_radii({0.01f}));

    // cleanup
    delete knn_graph;
    delete delaunay_graph;
    delete global_graph;
    return true;
}