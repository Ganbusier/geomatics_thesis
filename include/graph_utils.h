#include <easy3d/algo/delaunay_3d.h>
#include <easy3d/core/graph.h>
#include <easy3d/fileio/graph_io.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/kdtree/kdtree_search_eth.h>
#include <queue>
#include <unordered_set>

using namespace easy3d;

// declare hash function for std::pair<int, int> to use in std::unordered_map
namespace std {
    template<> struct hash<std::pair<int, int>> {
        size_t operator()(const std::pair<int, int>& p) const {
            return hash<long long>()( ((long long)p.first) ^ (((long long)p.second) << 32) );
        }
    };
}

namespace graph_utils {

void save_graph(Graph* graph, const std::string& filename) {
    if (GraphIO::save(filename, graph)) {
        LOG(INFO) << "Graph saved successfully to " << filename;
    } else {
        LOG(ERROR) << "Failed to save graph to " << filename;
    }
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
              << "m) from Delaunay graph, " << "total edges: " << combined_graph->n_edges();
    return combined_graph;
}

// a method to construct the dual-graph from a graph, just for information, not really used
// in the graph cut optimization i think, the original graph is enough
Graph* construct_dual_graph(Graph* graph) {
    Graph* dual_graph = new Graph;

    // map from edge index to vertex in dual graph
    std::map<int, Graph::Vertex> edge_to_vertex;

    // step 1: create a vertex for each edge (use the midpoint of the edge as the vertex position)
    for (const auto& e : graph->edges()) {
        auto source = graph->source(e);
        auto target = graph->target(e);
        auto source_pos = graph->position(source);
        auto target_pos = graph->position(target);

        // calculate the midpoint of the edge as the position of the vertex in the dual graph
        vec3 midpoint = (source_pos + target_pos) * 0.5f;
        auto dual_vertex = dual_graph->add_vertex(midpoint);

        // store the edge index to the dual vertex mapping
        edge_to_vertex[e.idx()] = dual_vertex;
    }

    // step 2: connect all dual vertices that are incident to the same original vertex
    // original vertex -> list of edges incident to the vertex
    std::map<int, std::vector<int>> vertex_to_edges;

    // collect the edges incident to each vertex
    for (const auto& e : graph->edges()) {
        auto source = graph->source(e);
        auto target = graph->target(e);

        vertex_to_edges[source.idx()].push_back(e.idx());
        vertex_to_edges[target.idx()].push_back(e.idx());
    }

    // create edges between dual vertices that are incident to the same original vertex
    for (const auto& [vertex_idx, edges] : vertex_to_edges) {
        // for each pair of edges incident to the same vertex, add an edge in the dual graph
        for (size_t i = 0; i < edges.size(); ++i) {
            for (size_t j = i + 1; j < edges.size(); ++j) {
                int edge1_idx = edges[i];
                int edge2_idx = edges[j];

                auto dual_vertex1 = edge_to_vertex[edge1_idx];
                auto dual_vertex2 = edge_to_vertex[edge2_idx];

                // add an edge in the dual graph
                dual_graph->add_edge(dual_vertex1, dual_vertex2);
            }
        }
    }

    LOG(INFO) << "Dual graph constructed with " << dual_graph->vertices_size() << " vertices and "
              << dual_graph->edges_size() << " edges";

    return dual_graph;
}

// method to compute data costs (the cost to preserve) for all edges in the original graph (nodes in
// the dual graph)
std::vector<float> compute_data_costs(Graph* graph, PointCloud* cloud, float extension_factor = 2.0f,
                                    float inlier_search_radius = 1.0f,
                                    float inlier_prob_weight = 1.0f) {
    // get points
    LOG(INFO) << "Getting points from point cloud...";
    auto points_property = cloud->get_vertex_property<vec3>("v:point");
    std::vector<vec3> points;
    for (const auto& v : cloud->vertices()) {
        points.push_back(points_property[v]);
    }
    LOG(INFO) << "Points retrieved successfully.";

    // use kd-tree to compute the mean point spacing, which is used for extending the edges
    LOG(INFO) << "Computing mean point spacing...";
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

    // initialize data costs, inliers probability cost and edge length cost with 1.0f which is the
    // maximum possible cost
    std::vector<float> data_costs(graph->n_edges(), 1.0f);
    std::vector<float> inliers_probability_costs(graph->n_edges(), 1.0f);
    std::vector<float> edge_length_costs(graph->n_edges(), 1.0f);

    // ================ Data Costs Computation: inliers probability costs ================
    // step 1: for each edge in the graph, extend its endpoints by the twice of the mean spacing
    // step 2: find all inliers that have a vertical distance <= 1.0m and within the extended edge
    // step 3: for each inlier, compute the probability of it being an edge point using the gaussian
    //         distribution
    // step 4: compute the inliers probability cost of the edge as the 1.0 minus mean value of sum
    //         of the probabilities of its inliers

    LOG(INFO) << "Computing inliers probability costs...";

    for (const auto& e : graph->edges()) {
        auto source = graph->source(e);
        auto target = graph->target(e);
        auto source_pos = graph->position(source);
        auto target_pos = graph->position(target);

        // extend the edge endpoints use mean spacing
        vec3 direction = (target_pos - source_pos).normalize();
        vec3 extended_source_pos = source_pos - extension_factor * mean_spacing * direction;
        vec3 extended_target_pos = target_pos + extension_factor * mean_spacing * direction;

        // find all inliers that have a vertical distance <= 1.0m and within the extended edge
        std::vector<int> inliers;
        std::vector<float> squared_distances;
        tree.find_points_in_cylinder(extended_source_pos, extended_target_pos, inlier_search_radius,
                                     inliers, squared_distances);
        if (inliers.empty() || squared_distances.empty()) {
            continue;
        }

        // exclude source and target points
        std::vector<float> filtered_squared_distances;
        for (size_t i = 0; i < inliers.size(); ++i) {
            if (inliers[i] != source.idx() && inliers[i] != target.idx()) {
                filtered_squared_distances.push_back(squared_distances[i]);
            }
        }
        if (filtered_squared_distances.empty()) {
            continue;
        }

        // compute the probability of each inlier being an edge point using the gaussian
        // distribution
        std::vector<float> probabilities;
        for (const float& sqrd_distance : filtered_squared_distances) {
            // use absolute value to avoid negative squared distance values (due to floating point
            // precision issues maybe)
            float distance = sqrt(abs(sqrd_distance));
            // variance of the distance, controls the decay rate, smaller decay faster
            // when x = inlier_search_radius, the probability is epsilon (1e-6, close to 0)
            const float epsilon = 1e-6f;
            float sigma_squared =
                (-inlier_search_radius * inlier_search_radius) / (2.0f * log(epsilon));
            // normalized gaussian function, value is 1 when distance is 0, decays as distance
            // increases
            float probability = exp(-distance * distance / (2.0f * sigma_squared));
            probabilities.push_back(probability);
        }

        // use the weighted sum of the probabilities of its inliers to compute the inliers
        // probability cost
        float weighted_sum = 0.0f;
        float weight_sum = 0.0f;
        for (const auto& probability : probabilities) {
            weighted_sum += probability * probability;  // use probability value as weight
            weight_sum += probability;
        }
        // avoid division by zero
        float inliers_probability_term = (weight_sum > 0) ? weighted_sum / weight_sum : 0.0f;
        inliers_probability_costs[e.idx()] = 1.0f - inliers_probability_term;
    }
    LOG(INFO) << "Inliers probability costs computed successfully.";

    // log max, min, and mean inliers probability cost
    float max_inliers_probability_cost =
        *std::max_element(inliers_probability_costs.begin(), inliers_probability_costs.end());
    float min_inliers_probability_cost =
        *std::min_element(inliers_probability_costs.begin(), inliers_probability_costs.end());
    float mean_inliers_probability_cost = 0.0f;
    for (const auto& cost : inliers_probability_costs) {
        mean_inliers_probability_cost += cost;
    }
    mean_inliers_probability_cost /= inliers_probability_costs.size();
    LOG(INFO) << "Max inliers probability cost: " << max_inliers_probability_cost;
    LOG(INFO) << "Min inliers probability cost: " << min_inliers_probability_cost;
    LOG(INFO) << "Mean inliers probability cost: " << mean_inliers_probability_cost;

    // ================ Data Costs Computation: edge length costs ================
    // step 1: for each edge in the graph, perform range search on both end-points within 0.1m radius
    // step 2: loop, while end points search results are not empty, extend end points by mean-spacing
    // step 3: stop when end points search results are empty
    // step 2: use gaussian function to compute the edge length cost using the edge length after search
    // step 3: store the result into edge_length_costs

    LOG(INFO) << "Computing edge length costs...";

    for (const auto& e : graph->edges()) {
        auto source = graph->source(e);
        auto target = graph->target(e);
        auto source_pos = graph->position(source);
        auto target_pos = graph->position(target);
        auto direction = (target_pos - source_pos).normalize();
        float edge_length = (target_pos - source_pos).length();
        // =================== new edge length cost computation ===================
        std::queue<Graph::Edge> edge_queue;
        std::unordered_set<int> processed_edges;
        float final_edge_length = 0.0f;
        edge_queue.push(e);
        processed_edges.insert(e.idx());
        final_edge_length += edge_length;

        while (!edge_queue.empty()) {
            auto current_edge = edge_queue.front();
            edge_queue.pop();
            auto v1 = graph->source(current_edge);
            auto v2 = graph->target(current_edge);
            auto direction = graph->position(v2) - graph->position(v1);
            direction = direction.normalize();

            auto process_vertex = [&](Graph::Vertex v) {
                float min_angle = 10.0f;
                Graph::Edge best_edge;
                for (auto neighbor_edge : graph->edges(v)) {
                    if (processed_edges.count(neighbor_edge.idx())) continue;
                    processed_edges.insert(neighbor_edge.idx());
                    auto n_v1 = graph->source(neighbor_edge);
                    auto n_v2 = graph->target(neighbor_edge);
                    auto n_direction = graph->position(n_v2) - graph->position(n_v1);
                    n_direction = n_direction.normalize();
                    float cosine = dot(direction, n_direction);
                    float angle = acos(std::clamp(cosine, -1.0f, 1.0f)) * 180.0f / M_PI; // [0, 180] degrees
                    if (angle < min_angle) {
                        min_angle = angle;
                        best_edge = neighbor_edge;
                    }
                }
                if (best_edge.idx() != -1) {
                    edge_queue.push(best_edge);
                    auto v1 = graph->source(best_edge);
                    auto v2 = graph->target(best_edge);
                    auto length = (graph->position(v2) - graph->position(v1)).length();
                    final_edge_length += length;
                }
            };
            process_vertex(v1);
            process_vertex(v2);
        }
        float edge_length_cost = edge_length / final_edge_length;
        edge_length_costs[e.idx()] = edge_length_cost;

        // =================== old edge length cost computation ===================
        // auto final_source_pos = source_pos;
        // auto final_target_pos = target_pos;
        // float search_radius = 2.0f * mean_spacing;
        // float scale_factor = 5.0f;

        // bool process_source = true;
        // bool process_target = true;
        // do {
        //     std::vector<int> source_inliers;
        //     std::vector<int> target_inliers;
        //     auto current_source_pos = final_source_pos;
        //     auto current_target_pos = final_target_pos;
        //     auto next_source_pos = current_source_pos - scale_factor * mean_spacing * direction;
        //     auto next_target_pos = current_target_pos + scale_factor * mean_spacing * direction;

        //     if (process_source) {
        //         tree.find_points_in_cylinder(current_source_pos, next_source_pos, search_radius, source_inliers);
        //         process_source = source_inliers.size() > 1;
        //     }
        //     if (process_target) {
        //         tree.find_points_in_cylinder(current_target_pos, next_target_pos, search_radius, target_inliers);
        //         process_target = target_inliers.size() > 1;
        //     }
        //     if (!process_source && !process_target) break;

        //     if (process_source) {
        //         final_source_pos = next_source_pos;   
        //     }
        //     if (process_target) {
        //         final_target_pos = next_target_pos; 
        //     }
        // } while (process_source || process_target);

        // float final_edge_length = (final_target_pos - final_source_pos).length();
        // float edge_length_cost = edge_length / final_edge_length;
        // edge_length_costs[e.idx()] = edge_length_cost;
    }

    LOG(INFO) << "Edge length costs computed successfully.";

    // log max, min, and mean edge length cost
    float max_edge_length_cost =
        *std::max_element(edge_length_costs.begin(), edge_length_costs.end());
    float min_edge_length_cost =
        *std::min_element(edge_length_costs.begin(), edge_length_costs.end());
    float mean_edge_length_cost = 0.0f;
    for (const auto& cost : edge_length_costs) {
        mean_edge_length_cost += cost;
    }
    mean_edge_length_cost /= edge_length_costs.size();
    LOG(INFO) << "Max edge length cost: " << max_edge_length_cost;
    LOG(INFO) << "Min edge length cost: " << min_edge_length_cost;
    LOG(INFO) << "Mean edge length cost: " << mean_edge_length_cost;

    // ================ Data Costs Computation: final data costs ================
    for (size_t i = 0; i < graph->n_edges(); ++i) {
        // float w1 = inlier_prob_weight;
        // float w2 = 1.0f - w1;
        // data_costs[i] = w1 * inliers_probability_costs[i] + w2 * edge_length_costs[i];

        // another data cost computation method
        data_costs[i] = inliers_probability_costs[i] * edge_length_costs[i];
    }
    LOG(INFO) << "Final data costs computed successfully.";

    // log max, min, and mean data cost
    float max_data_cost = *std::max_element(data_costs.begin(), data_costs.end());
    float min_data_cost = *std::min_element(data_costs.begin(), data_costs.end());
    float mean_data_cost = 0.0f;
    for (const auto& cost : data_costs) {
        mean_data_cost += cost;
    }
    mean_data_cost /= data_costs.size();
    LOG(INFO) << "Max data cost: " << max_data_cost;
    LOG(INFO) << "Min data cost: " << min_data_cost;
    LOG(INFO) << "Mean data cost: " << mean_data_cost;
    return data_costs;
}

struct SmoothnessCost {
    int edge1_idx;
    int edge2_idx;
    float smoothness_cost = 1.0f;
};

// method to compute smoothness costs for all edges in the original graph (nodes in the dual graph)
std::vector<SmoothnessCost> compute_smoothness_costs(Graph* graph) {
    std::vector<SmoothnessCost> smoothness_costs;

    // add a global set to avoid duplicate processing of the same edge pair
    std::set<std::pair<int, int>> global_processed_pairs;

    // ============ Smoothness Costs Computation: angle costs and distance costs ================
    // step 1: for each edge in the graph, find its neighbors
    // step 2: for each neighbor, compute the angle cost using gaussian function
    // step 3: store the result into SmoothnessCost struct

    for (const auto& e : graph->edges()) {
        auto source = graph->source(e);
        auto target = graph->target(e);
        vec3 source_pos = graph->position(source);
        vec3 target_pos = graph->position(target);
        vec3 edge_vector = target_pos - source_pos;

        // lambda function to compute the angle costs of the edge and its neighbors
        auto process_vertex = [&](Graph::Vertex vertex) {
            for (auto neighbor : graph->edges(vertex)) {
                // skip itself
                if (neighbor.idx() == e.idx()) continue;

                // avoid duplicate processing of the same edge pair
                int min_idx = std::min(e.idx(), neighbor.idx());
                int max_idx = std::max(e.idx(), neighbor.idx());
                auto edge_pair = std::make_pair(min_idx, max_idx);
                if (global_processed_pairs.find(edge_pair) != global_processed_pairs.end())
                    continue;
                global_processed_pairs.insert(edge_pair);

                // compute the angle
                auto neighbor_source = graph->source(neighbor);
                auto neighbor_target = graph->target(neighbor);
                vec3 neighbor_vector =
                    graph->position(neighbor_target) - graph->position(neighbor_source);

                float cosine_value = dot(edge_vector, neighbor_vector) /
                                     (edge_vector.length() * neighbor_vector.length());
                // limit the value range to [-1,1] to avoid NaN from acos
                cosine_value = std::max(-1.0f, std::min(1.0f, cosine_value));

                // angle cost function: 1 - (cos(x))^10
                float smoothness_cost = 1.0f - pow(cosine_value, 10.0f);

                // store the result
                smoothness_costs.push_back(
                    {e.idx(), neighbor.idx(), smoothness_cost});
            }
        };

        process_vertex(source);
        process_vertex(target);
    }

    // compute the max, min, and mean of the smoothness costs
    float max_smoothness_cost = 0.0f;
    float min_smoothness_cost = std::numeric_limits<float>::max();
    float mean_smoothness_cost = 0.0f;
    for (const auto& sc : smoothness_costs) {
        if (sc.smoothness_cost > max_smoothness_cost) max_smoothness_cost = sc.smoothness_cost;
        if (sc.smoothness_cost < min_smoothness_cost) min_smoothness_cost = sc.smoothness_cost;
        mean_smoothness_cost += sc.smoothness_cost;
    }
    mean_smoothness_cost /= smoothness_costs.size();
    LOG(INFO) << "Max smoothness cost: " << max_smoothness_cost;
    LOG(INFO) << "Min smoothness cost: " << min_smoothness_cost;
    LOG(INFO) << "Mean smoothness cost: " << mean_smoothness_cost;

    return smoothness_costs;
}
}  // namespace graph_utils
