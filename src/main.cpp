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

#include "graph_utils.h"

// define Graph as GCO_Graph to avoid conflict with easy3d::Graph when compiling
#define Graph GCO_Graph
#include "GCoptimization.h"
#undef Graph

using namespace easy3d;
using namespace rerun::demo;
using namespace graph_utils;

std::vector<Drawable*> drawables;  // store drawables added to the viewer

// function declarations
bool offset_xyz(Viewer* viewer, Model* model);
bool edge_length_test(Viewer* viewer, Model* model);  // test edge length costs
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
    offset_xyz(&viewer, model);

    // set up rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_impostor_type(PointsDrawable::PLAIN);
    drawable->set_point_size(3.0f);

    // set usage instructions
    viewer.set_usage("'Ctrl + g': run gco approach\n"
                     "'Ctrl + e': test edge length costs");

    // bind functions to keys
    viewer.bind(run_gco, model, Viewer::KEY_G, Viewer::MODIF_CTRL);
    viewer.bind(edge_length_test, model, Viewer::KEY_E, Viewer::MODIF_CTRL);

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

bool edge_length_test(Viewer* viewer, Model* model) {
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

    // ============================= test new edge length costs ========================================
    const auto rr = rerun::RecordingStream("Edge length Test Logger");
    rr.spawn().exit_on_failure();

    // log points
    std::vector<rerun::Position3D> rr_points;
    for (const auto& p : points) {
        rr_points.push_back(
            {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)});
    }

    // log global graph edges to rerun
    std::vector<rerun::Collection<rerun::Vec3D>> global_graph_edges;
    for (const auto& e : global_graph->edges()) {
        auto source = global_graph->source(e);
        auto target = global_graph->target(e);
        auto source_pos = global_graph->position(source); 
        auto target_pos = global_graph->position(target);
        rerun::Collection<rerun::Vec3D> global_graph_edge = {
            {static_cast<float>(source_pos.x), static_cast<float>(source_pos.y), static_cast<float>(source_pos.z)},
            {static_cast<float>(target_pos.x), static_cast<float>(target_pos.y), static_cast<float>(target_pos.z)}
        };
        global_graph_edges.push_back(global_graph_edge);
    }

    auto tree = KdTreeSearch_ETH(cloud);
    float mean_spacing = 0.0f;
    int count = 0;

    // find the second nearest neighbor for each point
    for (const auto& v : cloud->vertices()) {
        std::vector<int> nn_indices;
        std::vector<float> nn_distances;
        tree.find_closest_k_points(points[v.idx()], 2, nn_indices, nn_distances);
        if (nn_indices.size() > 1) {
            // the first neighbor is the point itself, the second neighbor is the nearest other
            // point
            mean_spacing += nn_distances[1];
            count++;
        }
    }
    // safe check to avoid division by zero
    if (count > 0) {
        mean_spacing /= count;
        mean_spacing = sqrt(mean_spacing);
        LOG(INFO) << "Mean point spacing: " << mean_spacing << "m (from " << count << " points)";
    } else {
        // use default value
        mean_spacing = 0.1f;
        LOG(INFO) << "Warning: Unable to compute mean point spacing, using default value: "
                  << mean_spacing << "m";
    }

    std::vector<rerun::Collection<rerun::Vec3D>> extended_edges;
    for (const auto& e : global_graph->edges()) {
        auto source = global_graph->source(e);
        auto target = global_graph->target(e);
        auto source_pos = global_graph->position(source);
        auto target_pos = global_graph->position(target);
        auto direction = (target_pos - source_pos).normalize();

        auto final_source_pos = source_pos;
        auto final_target_pos = target_pos;
        float search_radius = 0.1f;
        float scale_factor = 10.0f;

        bool process_source = true;
        bool process_target = true;
        do {
            std::vector<int> source_inliers;
            std::vector<int> target_inliers;
            auto current_source_pos = final_source_pos;
            auto current_target_pos = final_target_pos;
            auto next_source_pos = current_source_pos - scale_factor * mean_spacing * direction;
            auto next_target_pos = current_target_pos + scale_factor * mean_spacing * direction;

            if (process_source) {
                tree.find_points_in_cylinder(current_source_pos, next_source_pos, search_radius, source_inliers);
                process_source = source_inliers.size() > 2;
            }
            if (process_target) {
                tree.find_points_in_cylinder(current_target_pos, next_target_pos, search_radius, target_inliers);
                process_target = target_inliers.size() > 2;
            }
            if (!process_source && !process_target) break;

            if (process_source) {
                final_source_pos = next_source_pos;   
            }
            if (process_target) {
                final_target_pos = next_target_pos; 
            }
        } while (process_source || process_target);

        rerun::Collection<rerun::Vec3D> extended_edge = {
            {static_cast<float>(final_source_pos.x), static_cast<float>(final_source_pos.y), static_cast<float>(final_source_pos.z)},
            {static_cast<float>(final_target_pos.x), static_cast<float>(final_target_pos.y), static_cast<float>(final_target_pos.z)}
        };
        extended_edges.push_back(extended_edge);
    }

    rr.log("points", rerun::Points3D(rr_points).with_radii({0.05f}));
    rr.log("global graph edges", rerun::LineStrips3D(global_graph_edges).with_radii({0.02f}));
    rr.log("extended graph edges", rerun::LineStrips3D(extended_edges).with_radii({0.02f}));

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

    // set data costs
    std::vector<int> data_costs = compute_data_costs(global_graph, cloud, 2.0f, 1.0f,
                                                     1.0f);  // this is the cost to preserve an edge
    int max_data_cost = *std::max_element(data_costs.begin(), data_costs.end());
    int data_cost_scale_factor = 30;
    for (size_t i = 0; i < global_graph->n_edges(); ++i) {
        // the cost to remove an edge
        gc->setDataCost(i, 0, data_cost_scale_factor * (max_data_cost - data_costs[i]));
        // the cost to preserve an edge
        gc->setDataCost(i, 1, data_cost_scale_factor * data_costs[i]);
    }

    // set neighbors and smoothness costs
    std::vector<SmoothnessCost> smoothness_costs = compute_smoothness_costs(global_graph);
    LOG(INFO) << "Smoothness costs size: " << smoothness_costs.size();

    // compute neighbor-pair weights, lower the cost, higher the weight
    for (const auto& sc : smoothness_costs) {
        gc->setNeighbors(sc.edge1_idx, sc.edge2_idx, 100 - sc.smoothness_cost);
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
    std::vector<rerun::Collection<rerun::Vec3D>> preserved_edges;
    std::vector<rerun::Collection<rerun::Vec3D>> removed_edges;
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
            preserved_edges.push_back(strip);
        } else {
            removed_edges.push_back(strip);
        }
    }

    LOG(INFO) << "Preserved edges: " << preserved_edges.size();
    LOG(INFO) << "Removed edges: " << removed_edges.size();
    rr.log("preserved_edges", rerun::LineStrips3D(preserved_edges).with_radii({0.02f}));
    rr.log("removed_edges", rerun::LineStrips3D(removed_edges).with_radii({0.01f}));

    delete gc;
    delete knn_graph;
    delete delaunay_graph;
    delete global_graph;

    return true;
}
