#ifndef SEGMENTS_3_H
#define SEGMENTS_3_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/OSQP_quadratic_program_traits.h>
#include <CGAL/Shape_regularization.h>
#include <CGAL/Shape_regularization/regularize_segments.h>
#include <easy3d/core/graph.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <tuple>
#include <vector>

using namespace easy3d;

namespace segments_3 {

// forward declarations
class Neighbor_query_3;
class Angle_regularization_3;
struct Segment_3;

// basic type definitions
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Line_3 = Kernel::Line_3;
using Segments = std::vector<Segment_3>;

// QP solver type definition
using Quadratic_program = CGAL::OSQP_quadratic_program_traits<FT>;

// 3D regularizer type definition
using Regularizer_3 =
    CGAL::Shape_regularization::QP_regularization<Kernel, Segments, Neighbor_query_3,
                                                  Angle_regularization_3, Quadratic_program>;

// 3D line segment structure
struct Segment_3 {
    Point_3 source;
    Point_3 target;
    Vector_3 direction;  // normalized direction vector

    Segment_3(const Point_3& s, const Point_3& t) : source(s), target(t) {
        direction = (target - source);
        FT length = std::sqrt(direction.squared_length());
        if (length > 0) {
            direction = direction / length;
        }
    }

    // get segment length
    FT length() const { return std::sqrt(squared_length()); }

    // get segment direction vector (normalized)
    Vector_3 get_direction() const { return direction; }

    // get squared length of the segment
    FT squared_length() const { return (target - source).squared_length(); }
};

// class for neighbor query using the graph structure
class Neighbor_query_3 {
private:
    const Graph* graph;            // graph structure
    Segments segments;             // segments generated from the graph
    std::vector<std::vector<std::size_t>> groups; // predefined groups
    std::map<Graph::Edge, std::size_t> edge_to_segment;  // map from graph edges to segment indices
    std::vector<Graph::Edge> segment_to_edge;            // map from segment indices to graph edges

public:
    // constructor that only receives a graph structure
    Neighbor_query_3(const Graph* g) : graph(g) {
        if (graph) {
            // build segments from the graph
            build_segments_from_graph();
        } else {
            LOG(WARNING) << "No graph structure provided, neighbor query will be unavailable";
        }
    }

    // get the internally generated segments
    const Segments& get_segments() const {
        return segments;
    }

    // build segments from the graph
    void build_segments_from_graph() {
        LOG(INFO) << "Building segments from graph structure...";
        segments.clear();
        edge_to_segment.clear();
        segment_to_edge.clear();

        if (!graph) {
            LOG(WARNING) << "Graph structure is empty, cannot build segments";
            return;
        }

        size_t edge_count = 0;
        // iterate over all edges in the graph
        for (auto e : graph->edges()) {
            auto source = graph->source(e);
            auto target = graph->target(e);
            auto source_pos = graph->position(source);
            auto target_pos = graph->position(target);

            // create CGAL points
            Point_3 s(source_pos.x, source_pos.y, source_pos.z);
            Point_3 t(target_pos.x, target_pos.y, target_pos.z);

            // create and add the segment
            segments.emplace_back(s, t);
            
            // record the mapping from graph edges to segment indices
            edge_to_segment[e] = edge_count;
            // record the reverse mapping from segment indices to graph edges
            segment_to_edge.push_back(e);
            
            edge_count++;
        }

        LOG(INFO) << "Built " << segments.size() << " segments from the graph";
    }

    // query the neighbors of a segment
    void operator()(const std::size_t query_index, std::vector<std::size_t>& neighbors) {
        neighbors.clear();

        // check predefined groups first
        for (const auto& group : groups) {
            auto it = std::find(group.begin(), group.end(), query_index);
            if (it != group.end()) {
                // if in the group, return all other segments in the group
                for (const auto& idx : group) {
                    if (idx != query_index) {
                        neighbors.push_back(idx);
                    }
                }
                return;
            }
        }

        // if no graph structure or query index is invalid, return empty result
        if (!graph || segments.empty() || query_index >= segment_to_edge.size()) {
            LOG(WARNING) << "No valid graph structure for neighbor query or query index is invalid";
            return;
        }

        // directly get the graph edge corresponding to the query index via reverse mapping
        Graph::Edge query_edge = segment_to_edge[query_index];
        
        // get the source and target vertices of the edge
        auto source = graph->source(query_edge);
        auto target = graph->target(query_edge);

        // collect adjacent edges
        std::set<Graph::Edge> neighbor_edges;

        // process the neighbors of the source vertex
        for (auto e : graph->edges(source)) {
            if (e != query_edge) {
                neighbor_edges.insert(e);
            }
        }

        // process the neighbors of the target vertex
        for (auto e : graph->edges(target)) {
            if (e != query_edge) {
                neighbor_edges.insert(e);
            }
        }

        // convert adjacent edges to segment indices
        for (const auto& e : neighbor_edges) {
            auto it = edge_to_segment.find(e);
            if (it != edge_to_segment.end()) {
                neighbors.push_back(it->second);
            }
        }

        // remove duplicates
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }

    // add a predefined group
    void add_group(const std::vector<std::size_t>& group) { 
        groups.push_back(group); 
    }

    // clear all predefined groups
    void clear_groups() { 
        groups.clear();
    }
};

// angle regularization class (same as the original logic)
class Angle_regularization_3 {
private:
    Segments& segments;
    FT max_angle;
    std::vector<std::vector<std::size_t>> precomputed_neighbors; // store precomputed neighbor relationships
    const Graph* graph; // store the graph structure

public:
    // constructor that receives segments and graph pointer
    Angle_regularization_3(Segments& segs, const Graph* g, const FT max_angle_in_degree = FT(10))
        : segments(segs), graph(g), max_angle(max_angle_in_degree * CGAL_PI / FT(180)) {
        if (graph) {
            // precompute neighbor relationships using the graph
            precompute_neighbors_from_graph();
        } else {
            LOG(WARNING) << "No graph structure provided for angle regularization, neighbors will not be precomputed";
            return;
        }
    }

    // constructor that only receives segments (for backward compatibility)
    Angle_regularization_3(Segments& segs, const FT max_angle_in_degree = FT(10))
        : segments(segs), graph(nullptr), max_angle(max_angle_in_degree * CGAL_PI / FT(180)) {
        // initialize empty precomputed neighbors
        precomputed_neighbors.resize(segments.size());
        LOG(WARNING) << "No graph structure provided for angle regularization, neighbors will not be precomputed";
    }

    // precompute neighbor relationships using the graph structure
    void precompute_neighbors_from_graph() {
        precomputed_neighbors.resize(segments.size());
        
        // create a graph-based neighbor query
        Neighbor_query_3 neighbor_query(graph);
        
        LOG(INFO) << "Precomputing neighbor relationships for " << segments.size() << " segments using graph structure";
        
        // map segment indices to graph edges
        std::map<std::size_t, Graph::Edge> segment_to_edge;
        std::size_t idx = 0;
        for (auto e : graph->edges()) {
            if (idx < segments.size()) {
                segment_to_edge[idx++] = e;
            }
        }
        
        // for each segment, find its neighbors
        for (std::size_t i = 0; i < segments.size(); ++i) {
            neighbor_query(i, precomputed_neighbors[i]);
        }
        
        LOG(INFO) << "Neighbor precomputation completed using graph structure";
    }

    // return the maximum allowed angle change (in radians)
    FT bound(const std::size_t) const { return max_angle; }

    // calculate the target angle difference between two segments
    FT target(const std::size_t i, const std::size_t j) const {
        const Vector_3& dir_i = segments[i].get_direction();
        const Vector_3& dir_j = segments[j].get_direction();

        // calculate the angle between two direction vectors
        FT dot_product = dir_i * dir_j;
        // ensure the dot product is in the range of [-1,1]
        dot_product = std::max(std::min(dot_product, FT(1)), FT(-1));

        // if the angle is close to 0 or 180 degrees, return the angle to adjust
        FT angle = std::acos(std::abs(dot_product));
        if (angle < max_angle) {
            return angle;
        }
        return FT(0);
    }

    // update the direction of a segment based on the calculation result
    void update(const std::vector<FT>& angles) {
        for (std::size_t i = 0; i < segments.size(); ++i) {
            // use the Rodrigues rotation formula to update the direction
            FT angle = angles[i];
            if (std::abs(angle) > FT(0)) {
                // get the current segment direction
                Vector_3 current_dir = segments[i].get_direction();

                // use precomputed neighbors instead of querying again
                const std::vector<std::size_t>& segment_neighbors = precomputed_neighbors[i];

                // find the neighbor with smallest angle difference (not too small and not perpendicular)
                std::size_t best_neighbor_idx = segment_neighbors.size();
                FT best_angle_diff = FT(CGAL_PI);  // initialize with maximum possible angle
                Vector_3 best_neighbor_dir;

                for (const auto& j : segment_neighbors) {
                    const Vector_3& neighbor_dir = segments[j].get_direction();

                    // calculate angle with neighbor
                    FT dot = std::abs(current_dir * neighbor_dir);
                    dot = std::max(std::min(dot, FT(1)), FT(-1));
                    FT angle_with_neighbor = std::acos(dot);

                    // skip if angle is too small (nearly parallel) or too large (nearly perpendicular)
                    if (angle_with_neighbor < FT(0.1) ||  // ~5.7 degrees
                        std::abs(angle_with_neighbor - CGAL_PI / 2) < FT(0.1)) {
                        continue;
                    }

                    // check if this is the best neighbor so far
                    if (angle_with_neighbor < best_angle_diff) {
                        best_angle_diff = angle_with_neighbor;
                        best_neighbor_idx = j;
                        best_neighbor_dir = neighbor_dir;
                    }
                }

                // determine rotation axis based on the best neighbor if available
                Vector_3 rotation_axis;
                if (best_neighbor_idx < segment_neighbors.size()) {
                    // use the best neighbor direction to create rotation axis
                    rotation_axis = CGAL::cross_product(current_dir, best_neighbor_dir);

                    if (rotation_axis.squared_length() > FT(0.000001)) {
                        rotation_axis = rotation_axis / std::sqrt(rotation_axis.squared_length());
                    } else {
                        // fallback to default method if cross product is too small
                        rotation_axis = CGAL::cross_product(current_dir, Vector_3(FT(0), FT(0), FT(1)));

                        if (rotation_axis.squared_length() < FT(0.000001)) {
                            rotation_axis = CGAL::cross_product(current_dir, Vector_3(FT(0), FT(1), FT(0)));
                        }
                        rotation_axis = rotation_axis / std::sqrt(rotation_axis.squared_length());
                    }
                } else {
                    // fallback to default method if no suitable neighbors found
                    rotation_axis = CGAL::cross_product(current_dir, Vector_3(FT(0), FT(0), FT(1)));

                    if (rotation_axis.squared_length() < FT(0.000001)) {
                        rotation_axis = CGAL::cross_product(current_dir, Vector_3(FT(0), FT(1), FT(0)));
                    }
                    rotation_axis = rotation_axis / std::sqrt(rotation_axis.squared_length());
                }

                // apply the Rodrigues rotation formula
                FT cos_angle = std::cos(angle);
                FT sin_angle = std::sin(angle);

                Vector_3 new_dir =
                    current_dir * cos_angle +
                    CGAL::cross_product(rotation_axis, current_dir) * sin_angle +
                    rotation_axis * (rotation_axis * current_dir) * (FT(1) - cos_angle);

                // update the segment endpoints, using the midpoint as the rotation center
                FT length = segments[i].length();

                // calculate the midpoint
                Point_3 midpoint((segments[i].source.x() + segments[i].target.x()) / 2,
                                (segments[i].source.y() + segments[i].target.y()) / 2,
                                (segments[i].source.z() + segments[i].target.z()) / 2);

                // update the two endpoints, using the midpoint as the rotation center
                segments[i].source = midpoint - (new_dir * length / 2);
                segments[i].target = midpoint + (new_dir * length / 2);
                segments[i].direction = new_dir;
            }
        }
    }
    
    // Directly regularize given segments while using graph structure for neighbor relationships
    static void regularize(Segments& segments, const Graph* graph, const FT max_angle_degree = FT(10)) {
        LOG(INFO) << "Starting angle regularization with graph structure on " << segments.size() << " segments, max angle: " << max_angle_degree << "°";
        
        // Create neighbor query object based on graph structure
        Neighbor_query_3 neighbor_query(graph);
        
        // Create angle regularization object with graph structure
        Angle_regularization_3 angle_regularization(segments, graph, max_angle_degree);
        
        // Create QP solver
        Quadratic_program qp;
        
        // Create and execute regularization
        Regularizer_3 regularizer(segments, neighbor_query, angle_regularization, qp);
        regularizer.regularize();
        
        LOG(INFO) << "Angle regularization with graph structure completed";
    }
};

// for backward compatibility, keep the tool class for converting graph to segments
class Graph_to_Segments {
public:
    // convert the graph to segments
    static Segments convert(const Graph* graph) {
        LOG(INFO) << "converting the graph structure to segments...";
        Segments segments;
        
        if (!graph) {
            LOG(ERROR) << "the input graph structure is empty";
            return segments;
        }
        
        // iterate over all edges in the graph to create segments
        for (auto e : graph->edges()) {
            auto source = graph->source(e);
            auto target = graph->target(e);
            auto start = graph->position(source);
            auto end = graph->position(target);
            
            Point_3 s(start.x, start.y, start.z);
            Point_3 t(end.x, end.y, end.z);
            
            segments.emplace_back(s, t);
        }
        
        LOG(INFO) << "converted " << segments.size() << " segments";
        return segments;
    }
};

} // namespace segments_3

#endif // SEGMENTS_3_H

