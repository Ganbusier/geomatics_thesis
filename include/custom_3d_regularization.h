#ifndef CUSTOM_3D_REGULARIZATION_H
#define CUSTOM_3D_REGULARIZATION_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Shape_regularization.h>
#include <CGAL/OSQP_quadratic_program_traits.h>
#include <CGAL/Shape_regularization/regularize_segments.h>
#include <vector>
#include <cmath>

namespace custom_3d {

// Basic type definitions
using Kernel = CGAL::Simple_cartesian<double>;
using FT = Kernel::FT;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Line_3 = Kernel::Line_3;

// 3D line segment structure
struct Segment_3 {
    Point_3 source;
    Point_3 target;
    Vector_3 direction;  // Normalized direction vector

    Segment_3(const Point_3& s, const Point_3& t) 
        : source(s), target(t) {
        direction = (target - source);
        FT length = std::sqrt(direction.squared_length());
        if (length > 0) {
            direction = direction / length;
        }
    }

    // Get segment length
    FT length() const {
        return std::sqrt(squared_length());
    }

    // Get segment direction vector (normalized)
    Vector_3 get_direction() const {
        return direction;
    }

    // Get squared length of the segment
    FT squared_length() const {
        return (target - source).squared_length();
    }
};

using Segments = std::vector<Segment_3>;

// 3D neighborhood query class
class Neighbor_query_3 {
private:
    const Segments& segments;
    std::vector<std::vector<std::size_t>> groups;

public:
    Neighbor_query_3(const Segments& segs) : segments(segs) {}

    void operator()(
        const std::size_t query_index,
        std::vector<std::size_t>& neighbors) {
        
        neighbors.clear();
        
        // If predefined groups exist, use neighbors within the group
        for (const auto& group : groups) {
            auto it = std::find(group.begin(), group.end(), query_index);
            if (it != group.end()) {
                for (const auto& idx : group) {
                    if (idx != query_index) {
                        neighbors.push_back(idx);
                    }
                }
                return;
            }
        }

        // Otherwise, use a simple distance threshold to determine neighbors
        const FT distance_threshold = FT(5.0); // Can be adjusted as needed
        const auto& query_segment = segments[query_index];
        
        for (std::size_t i = 0; i < segments.size(); ++i) {
            if (i == query_index) continue;
            
            const auto& current_segment = segments[i];
            // Calculate distance between midpoints of two segments
            Point_3 query_midpoint(
                (query_segment.source.x() + query_segment.target.x()) / 2,
                (query_segment.source.y() + query_segment.target.y()) / 2,
                (query_segment.source.z() + query_segment.target.z()) / 2
            );
            
            Point_3 current_midpoint(
                (current_segment.source.x() + current_segment.target.x()) / 2,
                (current_segment.source.y() + current_segment.target.y()) / 2,
                (current_segment.source.z() + current_segment.target.z()) / 2
            );
            
            if (CGAL::squared_distance(query_midpoint, current_midpoint) < distance_threshold * distance_threshold) {
                neighbors.push_back(i);
            }
        }
    }

    void add_group(const std::vector<std::size_t>& group) {
        groups.push_back(group);
    }

    void clear() {
        groups.clear();
    }
};

// 3D angle regularization class
class Angle_regularization_3 {
private:
    Segments& segments;
    FT max_angle;

public:
    Angle_regularization_3(
        Segments& segs,
        const FT max_angle_in_degree = FT(10)) 
        : segments(segs), max_angle(max_angle_in_degree * CGAL_PI / FT(180)) {}

    // Return maximum allowed angle change (in radians)
    FT bound(const std::size_t) const {
        return max_angle;
    }

    // Calculate target angle difference between two segments
    FT target(const std::size_t i, const std::size_t j) const {
        const Vector_3& dir_i = segments[i].get_direction();
        const Vector_3& dir_j = segments[j].get_direction();
        
        // Calculate angle between two direction vectors
        FT dot_product = dir_i * dir_j;
        // Ensure dot product is within [-1,1]
        dot_product = std::max(std::min(dot_product, FT(1)), FT(-1));
        
        // If angle is close to 0 or 180 degrees, return the angle that needs adjustment
        FT angle = std::acos(std::abs(dot_product));
        if (angle < max_angle) {
            return angle;
        }
        return FT(0);
    }

    // Update segment directions based on calculation results
    void update(const std::vector<FT>& angles) {
        for (std::size_t i = 0; i < segments.size(); ++i) {
            // Update direction using Rodrigues rotation formula
            FT angle = angles[i];
            if (std::abs(angle) > FT(0)) {
                // Construct rotation axis (perpendicular to current direction)
                Vector_3 current_dir = segments[i].get_direction();
                Vector_3 rotation_axis = CGAL::cross_product(
                    current_dir,
                    Vector_3(FT(0), FT(0), FT(1)));
                
                if (rotation_axis.squared_length() < FT(0.000001)) {
                    rotation_axis = CGAL::cross_product(
                        current_dir,
                        Vector_3(FT(0), FT(1), FT(0)));
                }
                
                rotation_axis = rotation_axis / std::sqrt(rotation_axis.squared_length());
                
                // Apply Rodrigues rotation formula
                FT cos_angle = std::cos(angle);
                FT sin_angle = std::sin(angle);
                
                Vector_3 new_dir = 
                    current_dir * cos_angle +
                    CGAL::cross_product(rotation_axis, current_dir) * sin_angle +
                    rotation_axis * (rotation_axis * current_dir) * (FT(1) - cos_angle);
                
                // Update segment endpoints
                FT length = segments[i].length();
                segments[i].target = segments[i].source + new_dir * length;
                segments[i].direction = new_dir;
            }
        }
    }
};

// 3D position offset regularization class
class Offset_regularization_3 {
private:
    Segments& segments;
    FT max_offset;
    FT merge_threshold;  // Threshold for merging segments
    std::vector<std::vector<std::size_t>> parallel_groups;
    std::vector<bool> merged;  // Track merged segments

public:
    Offset_regularization_3(
        Segments& segs,
        const FT max_offset_value = FT(0.5),
        const FT merge_threshold_value = FT(0.1))  // Default merge threshold is 0.1
        : segments(segs), 
          max_offset(max_offset_value),
          merge_threshold(merge_threshold_value),
          merged(segs.size(), false) {}

    // Return maximum allowed offset
    FT bound(const std::size_t) const {
        return max_offset;
    }

    // Calculate offset difference between two parallel segments
    FT target(const std::size_t i, const std::size_t j) const {
        const auto& seg_i = segments[i];
        const auto& seg_j = segments[j];
        
        // Only process parallel segments
        FT dot_product = seg_i.direction * seg_j.direction;
        if (std::abs(std::abs(dot_product) - FT(1)) > FT(0.1)) {
            return FT(0);
        }

        // Calculate perpendicular distance from segment i to segment j
        Vector_3 diff = seg_j.source - seg_i.source;
        Vector_3 normal = CGAL::cross_product(seg_i.direction, 
            CGAL::cross_product(seg_i.direction, seg_j.direction));
        
        if (normal.squared_length() < FT(0.000001)) {
            return FT(0);
        }
        
        normal = normal / std::sqrt(normal.squared_length());
        FT distance = std::abs(diff * normal);
        
        return (distance < max_offset) ? distance : FT(0);
    }

    // Update segment positions based on calculation results
    void update(const std::vector<FT>& offsets) {
        // Reset merge flags
        std::fill(merged.begin(), merged.end(), false);
        
        // First apply offsets
        for (std::size_t i = 0; i < segments.size(); ++i) {
            if (merged[i]) continue;  // Skip merged segments
            
            FT offset = offsets[i];
            if (std::abs(offset) > FT(0)) {
                // Find translation direction (perpendicular to segment direction)
                Vector_3 current_dir = segments[i].direction;
                Vector_3 offset_dir = CGAL::cross_product(
                    current_dir,
                    Vector_3(FT(0), FT(0), FT(1)));
                
                if (offset_dir.squared_length() < FT(0.000001)) {
                    offset_dir = CGAL::cross_product(
                        current_dir,
                        Vector_3(FT(0), FT(1), FT(0)));
                }
                
                offset_dir = offset_dir / std::sqrt(offset_dir.squared_length());
                
                // Apply offset
                Vector_3 translation = offset_dir * offset;
                segments[i].source = segments[i].source + translation;
                segments[i].target = segments[i].target + translation;
            }
        }

        // Then check and merge close segments
        for (const auto& group : parallel_groups) {
            for (size_t i = 0; i < group.size(); ++i) {
                if (merged[group[i]]) continue;
                
                for (size_t j = i + 1; j < group.size(); ++j) {
                    if (merged[group[j]]) continue;
                    
                    const auto& seg_i = segments[group[i]];
                    const auto& seg_j = segments[group[j]];
                    
                    // Calculate distance between segments
                    Vector_3 diff = seg_j.source - seg_i.source;
                    Vector_3 normal = CGAL::cross_product(seg_i.direction, 
                        CGAL::cross_product(seg_i.direction, seg_j.direction));
                    
                    if (normal.squared_length() < FT(0.000001)) continue;
                    
                    normal = normal / std::sqrt(normal.squared_length());
                    FT distance = std::abs(diff * normal);
                    
                    // If distance is less than merge threshold, merge segments
                    if (distance < merge_threshold) {
                        // Use length of longer segment
                        FT length_i = seg_i.length();
                        FT length_j = seg_j.length();
                        FT new_length = std::max(length_i, length_j);
                        
                        // Calculate midpoint position
                        Point_3 mid_source(
                            (seg_i.source.x() + seg_j.source.x()) / 2,
                            (seg_i.source.y() + seg_j.source.y()) / 2,
                            (seg_i.source.z() + seg_j.source.z()) / 2
                        );
                        
                        // Update the longer segment
                        if (length_i >= length_j) {
                            segments[group[i]].source = mid_source;
                            segments[group[i]].target = mid_source + seg_i.direction * new_length;
                            merged[group[j]] = true;
                        } else {
                            segments[group[j]].source = mid_source;
                            segments[group[j]].target = mid_source + seg_j.direction * new_length;
                            merged[group[i]] = true;
                        }
                    }
                }
            }
        }
    }

    // Add parallel segment group
    void add_group(const std::vector<std::size_t>& group) {
        parallel_groups.push_back(group);
    }

    // Clear all groups
    void clear_groups() {
        parallel_groups.clear();
        merged.clear();
        if (segments.size() > 0) {
            merged.resize(segments.size(), false);
        }
    }
};

// QP solver type definition
using Quadratic_program = CGAL::OSQP_quadratic_program_traits<FT>;

// 3D regularizer type definition
using Regularizer_3 = CGAL::Shape_regularization::QP_regularization<
    Kernel, Segments, Neighbor_query_3, Angle_regularization_3, Quadratic_program>;

// Define combined regularizer for sequential angle and position offset regularization
class Combined_regularization_3 {
public:
    struct Parameters {
        FT max_angle_degree;      // Maximum angle deviation (degrees)
        FT max_offset;            // Maximum position offset
        FT parallel_angle_degree; // Angle threshold for parallel determination (degrees)
        FT merge_threshold;       // Threshold for merging close parallel segments

        Parameters(
            FT angle_degree = FT(10.0),     // Default maximum angle deviation 10 degrees
            FT offset = FT(0.5),            // Default maximum offset 0.5
            FT parallel_degree = FT(5.7),    // Default parallel determination threshold 5.7 degrees
            FT merge_thresh = FT(0.1)        // Default merge threshold 0.1
        ) : max_angle_degree(angle_degree),
            max_offset(offset),
            parallel_angle_degree(parallel_degree),
            merge_threshold(merge_thresh)
        {}
    };

    static void regularize(
        Segments& segments,
        const Parameters& params = Parameters()
    ) {
        // Convert parallel determination angle to cosine value
        FT parallel_threshold = std::cos(params.parallel_angle_degree * CGAL_PI / FT(180));

        // Step 1: Angle regularization
        Neighbor_query_3 angle_neighbor_query(segments);
        Angle_regularization_3 angle_regularization(segments, params.max_angle_degree);
        Quadratic_program qp_angle;
        Regularizer_3 angle_regularizer(
            segments,
            angle_neighbor_query,
            angle_regularization,
            qp_angle
        );
        angle_regularizer.regularize();

        // Step 2: Find parallel segment groups
        std::vector<std::vector<std::size_t>> parallel_groups;
        
        for (std::size_t i = 0; i < segments.size(); ++i) {
            bool found_group = false;
            const Vector_3& dir_i = segments[i].direction;
            
            // Check if can join existing group
            for (auto& group : parallel_groups) {
                const Vector_3& group_dir = segments[group[0]].direction;
                FT dot_product = std::abs(dir_i * group_dir);
                if (std::abs(dot_product - FT(1)) < (FT(1) - parallel_threshold)) {
                    group.push_back(i);
                    found_group = true;
                    break;
                }
            }
            
            // If no suitable group found, create new group
            if (!found_group) {
                parallel_groups.push_back({i});
            }
        }

        // Step 3: Position offset regularization
        Neighbor_query_3 offset_neighbor_query(segments);
        Offset_regularization_3 offset_regularization(
            segments, 
            params.max_offset,
            params.merge_threshold  // Pass merge threshold to Offset_regularization_3
        );
        
        // Add parallel group information
        for (const auto& group : parallel_groups) {
            if (group.size() > 1) {
                offset_neighbor_query.add_group(group);
                offset_regularization.add_group(group);
            }
        }
        
        Quadratic_program qp_offset;
        using Offset_regularizer_3 = CGAL::Shape_regularization::QP_regularization<
            Kernel, Segments, Neighbor_query_3, Offset_regularization_3, Quadratic_program>;
            
        Offset_regularizer_3 offset_regularizer(
            segments,
            offset_neighbor_query,
            offset_regularization,
            qp_offset
        );
        offset_regularizer.regularize();
    }
};

} // namespace custom_3d

#endif // CUSTOM_3D_REGULARIZATION_H 