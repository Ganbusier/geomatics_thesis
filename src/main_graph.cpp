#include <easy3d/algo/delaunay_3d.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/core/graph.h>
#include <easy3d/core/model.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/kdtree/kdtree_search_eth.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/initializer.h>
#include <easy3d/util/resource.h>
#include <easy3d/viewer/viewer.h>

#include <filesystem>
#include <iostream>
#include <unordered_map>
#include <rerun.hpp>
#include <rerun/demo_utils.hpp>

#include "graph_utils.h"

// 定义vec3的哈希函数，用于unordered_map
struct vec3_hash {
    std::size_t operator()(const easy3d::vec3& v) const {
        return std::hash<float>()(v.x) ^ std::hash<float>()(v.y) ^ std::hash<float>()(v.z);
    }
};

// 定义vec3的等价操作符，用于unordered_map
bool operator==(const easy3d::vec3& lhs, const easy3d::vec3& rhs) {
    const float epsilon = 1e-6f;
    return std::abs(lhs.x - rhs.x) < epsilon && 
           std::abs(lhs.y - rhs.y) < epsilon && 
           std::abs(lhs.z - rhs.z) < epsilon;
}

// define Graph as GCO_Graph to avoid conflict with easy3d::Graph when compiling
#define Graph GCO_Graph
#include "GCoptimization.h"
#undef Graph

using namespace easy3d;
using namespace rerun::demo;
using namespace graph_utils;

// function declarations
bool offset_xyz(Viewer* viewer, Model* model);
bool testDataCost(Viewer* viewer, Model* model);  // test data costs
bool run_gco(Viewer* viewer, Model* model);

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
    // offset_xyz(&viewer, model);

    // set up rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_impostor_type(PointsDrawable::PLAIN);
    drawable->set_point_size(3.0f);

    // set usage instructions
    viewer.set_usage("'Ctrl + g': run gco approach\n"
                     "'Ctrl + i': test data costs.");

    // bind functions to keys
    viewer.bind(run_gco, model, Viewer::KEY_G, Viewer::MODIF_CTRL);
    viewer.bind(testDataCost, model, Viewer::KEY_I, Viewer::MODIF_CTRL);

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

bool testDataCost(Viewer* viewer, Model* model) {
    if (!viewer ||!model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model); 
    auto points_property = cloud->get_vertex_property<vec3>("v:point");

    // get points
    std::vector<vec3> points;
    for (const auto& v : cloud->vertices()) {
        points.push_back(points_property[v]);
    }

    // build knn graph
    int k_neighbors = 10;
    easy3d::Graph* knn_graph = build_knn_graph(cloud, k_neighbors);

    // build delaunay graph
    easy3d::Graph* delaunay_graph = build_delaunay_graph(cloud);

    // combine graphs
    const float max_edge_length = 2.0f;
    easy3d::Graph* global_graph = combine_graphs(knn_graph, delaunay_graph, max_edge_length);

    std::vector<float> data_costs = compute_data_costs(global_graph, cloud, 2.0f, 1.0f,
                                                     0.0f);  // this is the cost to preserve an edge

    const auto rr = rerun::RecordingStream("Data Cost Test Logger");
    rr.spawn().exit_on_failure();

    // log points
    std::vector<rerun::Position3D> rr_points;
    for (const auto& p : points) {
        rr_points.push_back(
            {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
    }

    // log global graph edges based on their data costs
    std::vector<rerun::Collection<rerun::Vec3D>> edges_less_than_10;
    std::vector<rerun::Collection<rerun::Vec3D>> edges_10_to_30;
    std::vector<rerun::Collection<rerun::Vec3D>> edges_30_to_50;
    std::vector<rerun::Collection<rerun::Vec3D>> edges_more_than_50;
    int iter = 0;
    for (const auto& e : global_graph->edges()) {
        auto source = global_graph->source(e);
        auto target = global_graph->target(e);
        auto source_pos = global_graph->position(source); 
        auto target_pos = global_graph->position(target);
        rerun::Collection<rerun::Vec3D> edge = {
            {static_cast<float>(source_pos.x), static_cast<float>(source_pos.y), static_cast<float>(source_pos.z)},
            {static_cast<float>(target_pos.x), static_cast<float>(target_pos.y), static_cast<float>(target_pos.z)}
        };
        if (data_costs[iter] <= 0.1) {
            edges_less_than_10.push_back(edge); 
        }
        else if (data_costs[iter] <= 0.3) {
            edges_10_to_30.push_back(edge); 
        }
        else if (data_costs[iter] <= 0.5) {
            edges_30_to_50.push_back(edge); 
        }
        else {
            edges_more_than_50.push_back(edge); 
        }
        iter++;
    }

    rr.log("points", rerun::Points3D(rr_points).with_radii({0.05f}));
    rr.log("data cost <=10", rerun::LineStrips3D(edges_less_than_10).with_radii({0.02f}));
    rr.log("10 < data cost <= 30", rerun::LineStrips3D(edges_10_to_30).with_radii({0.02f}));
    rr.log("30 < data cost <= 50", rerun::LineStrips3D(edges_30_to_50).with_radii({0.02f}));
    rr.log("50 < data cost", rerun::LineStrips3D(edges_more_than_50).with_radii({0.02f}));

    delete knn_graph;
    delete delaunay_graph;
    delete global_graph;

    return true;
}

bool run_gco(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto points_property = cloud->get_vertex_property<vec3>("v:point");

    // get points
    std::vector<vec3> points;
    for (const auto& v : cloud->vertices()) {
        points.push_back(points_property[v]);
    }

    // build knn graph
    int k_neighbors = 10;
    easy3d::Graph* knn_graph = build_knn_graph(cloud, k_neighbors);

    // build delaunay graph
    easy3d::Graph* delaunay_graph = build_delaunay_graph(cloud);

    // combine graphs
    const float max_edge_length = 2.0f;
    easy3d::Graph* global_graph = combine_graphs(knn_graph, delaunay_graph, max_edge_length);

    // construct dual graph
    // easy3d::Graph* dual_graph = construct_dual_graph(global_graph);
    
    // ================================= run GCO =================================
    int num_labels = 2;  // 2 labels: 0 and 1 --> 0: remove, 1: preserve
    GCoptimizationGeneralGraph* gc =
        new GCoptimizationGeneralGraph(global_graph->n_edges(), num_labels);
    
    int scale_factor = 100; // for both data costs and smoothness costs
    float lambda1 = 1.0f; // control the weight of the data costs
    float lambda2 = 0.1f; // control the weight of the smoothness costs

    // set data costs
    std::vector<float> data_costs = compute_data_costs(global_graph, cloud, 2.0f, 1.0f,
                                                     0.0f);  // this is the cost to preserve an edge
    for (size_t i = 0; i < global_graph->n_edges(); ++i) {
        // convert float to int with scale factor
        int dc_preserved = static_cast<int>(std::floor(lambda1 * data_costs[i] * scale_factor));
        int dc_removed = static_cast<int>(std::floor(lambda1 * (1.0f - data_costs[i]) * scale_factor));
        // the cost to remove an edge
        gc->setDataCost(i, 0, dc_removed);
        // the cost to preserve an edge
        gc->setDataCost(i, 1, dc_preserved);
    }

    // set neighbors and smoothness costs
    std::vector<SmoothnessCost> smoothness_costs = compute_smoothness_costs(global_graph);
    LOG(INFO) << "Smoothness costs size: " << smoothness_costs.size();

    // compute neighbor-pair weights, lower the cost, higher the weight
    for (const auto& sc : smoothness_costs) {
        float sc_scaled = sc.smoothness_cost * scale_factor;
        float nn_weight = scale_factor - sc_scaled;
        int neighbor_pair_weight = static_cast<int>(std::floor(nn_weight * lambda2)); 
        gc->setNeighbors(sc.edge1_idx, sc.edge2_idx, neighbor_pair_weight);
    }
    // heavily penalize different labels for low-angle-diff neighbor-pairs
    int V[4] = {0, 1, 1, 0};  // V[label1 + num_label*label2] --> V(0,0), V(1,0), V(0,1), V(1,1)
                              // must satisfy: V(0, 0) + V(1, 1) <= V(0,1) + V(1,0)
    gc->setSmoothCost(V); // initially the smooth cost will be: sum(w_i * V(0,0))

    LOG(INFO) << "Before optimization, energy: " << gc->compute_energy()
              << ", data cost: " << gc->giveDataEnergy()
              << ", smoothness cost: " << gc->giveSmoothEnergy();
    gc->expansion(99);
    LOG(INFO) << "After optimization, energy: " << gc->compute_energy()
              << ", data cost: " << gc->giveDataEnergy()
              << ", smoothness cost: " << gc->giveSmoothEnergy();

    // log preserved and removed edges to rerun
    const auto rr = rerun::RecordingStream("GCO Approach logger");
    rr.spawn().exit_on_failure();

    // log points
    std::vector<rerun::Position3D> rr_points;
    for (const auto& p : points) {
        rr_points.push_back(
            {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
    }
    rr.log("points", rerun::Points3D(rr_points));

    // log preserved and removed edges seperately
    std::vector<rerun::Collection<rerun::Vec3D>> rr_preserved_edges;
    std::vector<rerun::Collection<rerun::Vec3D>> rr_removed_edges;
    std::vector<std::vector<vec3>> preserved_edges;
    std::vector<std::vector<vec3>> removed_edges;
    for (const auto& e : global_graph->edges()) {
        int label = gc->whatLabel(e.idx());
        auto source = global_graph->source(e);
        auto target = global_graph->target(e);
        auto start = global_graph->position(source);
        auto end = global_graph->position(target);

        rerun::Collection<rerun::Vec3D> strip = {
            {static_cast<float>(start.x), static_cast<float>(start.y), static_cast<float>(start.z)},
            {static_cast<float>(end.x), static_cast<float>(end.y), static_cast<float>(end.z)}};

        if (label == 1) {
            rr_preserved_edges.push_back(strip);
            preserved_edges.push_back({start, end});
        } else {
            rr_removed_edges.push_back(strip);
            removed_edges.push_back({start, end});
        }
    }

    LOG(INFO) << "Preserved edges: " << rr_preserved_edges.size();
    LOG(INFO) << "Removed edges: " << rr_removed_edges.size();
    rr.log("preserved_edges", rerun::LineStrips3D(rr_preserved_edges).with_radii({0.02f}));
    rr.log("removed_edges", rerun::LineStrips3D(rr_removed_edges).with_radii({0.01f}));

    // 导出preserved_edges到.ply文件
    easy3d::Graph* preserved_graph = new easy3d::Graph;
    std::unordered_map<vec3, easy3d::Graph::Vertex, vec3_hash> point_to_vertex_preserved;

    // 为preserved_edges添加顶点和边
    for (const auto& edge : preserved_edges) {
        vec3 start(edge[0].x, edge[0].y, edge[0].z);
        vec3 end(edge[1].x, edge[1].y, edge[1].z);
        
        easy3d::Graph::Vertex v1, v2;
        
        // 检查起点是否已存在
        if (point_to_vertex_preserved.find(start) == point_to_vertex_preserved.end()) {
            v1 = preserved_graph->add_vertex(start);
            point_to_vertex_preserved[start] = v1;
        } else {
            v1 = point_to_vertex_preserved[start];
        }
        
        // 检查终点是否已存在
        if (point_to_vertex_preserved.find(end) == point_to_vertex_preserved.end()) {
            v2 = preserved_graph->add_vertex(end);
            point_to_vertex_preserved[end] = v2;
        } else {
            v2 = point_to_vertex_preserved[end];
        }
        
        preserved_graph->add_edge(v1, v2);
    }
    
    // 导出removed_edges到.ply文件
    easy3d::Graph* removed_graph = new easy3d::Graph;
    std::unordered_map<vec3, easy3d::Graph::Vertex, vec3_hash> point_to_vertex_removed;
    
    // 为removed_edges添加顶点和边
    for (const auto& edge : removed_edges) {
        vec3 start(edge[0].x, edge[0].y, edge[0].z);
        vec3 end(edge[1].x, edge[1].y, edge[1].z);
        
        easy3d::Graph::Vertex v1, v2;
        
        // 检查起点是否已存在
        if (point_to_vertex_removed.find(start) == point_to_vertex_removed.end()) {
            v1 = removed_graph->add_vertex(start);
            point_to_vertex_removed[start] = v1;
        } else {
            v1 = point_to_vertex_removed[start];
        }
        
        // 检查终点是否已存在
        if (point_to_vertex_removed.find(end) == point_to_vertex_removed.end()) {
            v2 = removed_graph->add_vertex(end);
            point_to_vertex_removed[end] = v2;
        } else {
            v2 = point_to_vertex_removed[end];
        }
        
        removed_graph->add_edge(v1, v2);
    }
    
    // save preserved_edges and removed_edges
    io::save_ply("preservedEdges.ply", preserved_graph, false);
    io::save_ply("removedEdges.ply", removed_graph, false);

    // also save knn_graph, delaunay_graph, global_graph
    io::save_ply("knnGraph.ply", knn_graph, false);
    io::save_ply("dtGraph.ply", delaunay_graph, false);
    io::save_ply("combineGraph.ply", global_graph, false);
    
    // 清理资源
    delete preserved_graph;
    delete removed_graph;

    delete gc;
    delete knn_graph;
    delete delaunay_graph;
    delete global_graph;

    return true;
}