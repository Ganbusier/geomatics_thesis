#ifndef CUSTOM_RANSAC_H
#define CUSTOM_RANSAC_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_set_3/IO.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <numeric>
#include <random>
#include <set>
#include <vector>

namespace custom_ransac {

// ===== basic type definitions =====
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using Line_3 = Kernel::Line_3;

// types for ransac point-normal pairs
using Point_with_normal = std::pair<Point_3, Vector_3>;
using Pwn_vector = std::vector<Point_with_normal>;
using Point_map = CGAL::First_of_pair_property_map<Point_with_normal>;
using Normal_map = CGAL::Second_of_pair_property_map<Point_with_normal>;

// efficient ransac related types
using Efficient_ransac_traits =
    CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map>;
using Efficient_ransac = CGAL::Shape_detection::Efficient_RANSAC<Efficient_ransac_traits>;
using Plane_shape = CGAL::Shape_detection::Plane<Efficient_ransac_traits>;
using Cylinder_shape = CGAL::Shape_detection::Cylinder<Efficient_ransac_traits>;

// plane detection result structure
struct Plane_result {
    Plane_3 plane;                            // detected plane
    std::vector<std::size_t> inlier_indices;  // indices of points belonging to the plane
    Pwn_vector points_with_normals;  // points and normals on the plane (part of pwn_vector)
};

// cylinder detection result structure
struct Cylinder_result {
    Point_3 center;                          // point on cylinder axis
    Vector_3 axis;                           // axis direction
    double radius;                           // cylinder radius
    std::vector<std::size_t> inlier_indices; // indices of points belonging to the cylinder
    Pwn_vector points_with_normals;          // points and normals on the cylinder
};

// ===== 2d ransac line segment detection =====

class Ransac_2d {
   public:
    struct Point {
        double x;
        double y;
    };

    struct Line {
        double a;  // ax + by + c = 0
        double b;
        double c;
        Point start;
        Point end;
        std::vector<size_t> inlier_indices;
    };

    struct Parameters {
        size_t max_iterations;   // maximum number of iterations
        size_t min_inliers;      // minimum number of inliers for valid line segment
        double tolerance;        // maximum distance of inliers to line
        double split_threshold;  // distance threshold for segment splitting
        double min_length;       // minimum line segment length
        double max_length;       // maximum line segment length

        Parameters(size_t max_iter = 1000, size_t min_inl = 5, double tol = 0.1,
                   double split_thres = 1.0, double min_len = 0.0, double max_len = 1e6)
            : max_iterations(max_iter),
              min_inliers(min_inl),
              tolerance(tol),
              split_threshold(split_thres),
              min_length(min_len),
              max_length(max_len) {}
    };

    // detect line segments in 2d point cloud
    std::vector<Line> detect(const std::vector<Point>& points,
                             const Parameters& params = Parameters()) {
        std::vector<Line> lines;
        std::vector<size_t> remaining_indices(points.size());
        std::iota(remaining_indices.begin(), remaining_indices.end(), 0);

        // adaptive inlier threshold (default is 10% of point cloud size)
        size_t inlier_thres =
            std::max(params.min_inliers, static_cast<size_t>(std::floor(points.size() * 0.1)));

        if (points.size() <= params.min_inliers) return lines;

        while (inlier_thres >= params.min_inliers) {
            LOG(INFO) << "inlier_thres: " << inlier_thres << " remaining_indices.size(): " << remaining_indices.size();
            std::vector<Line> candidate_lines;

            size_t iter = 0;
            while (iter < params.max_iterations && remaining_indices.size() >= inlier_thres) {
                // randomly select two points
                if (remaining_indices.size() < 2) break;

                std::vector<size_t> sample_indices(2);
                std::sample(remaining_indices.begin(), remaining_indices.end(),
                            sample_indices.begin(), 2, rng);
                const Point& p1 = points[sample_indices[0]];
                const Point& p2 = points[sample_indices[1]];

                // skip overlapping or too distant points
                double distance = std::hypot(p1.x - p2.x, p1.y - p2.y);
                if (distance < 1e-6) {
                    iter++;
                    continue;
                }

                // compute candidate line segment
                Line candidate_line = compute_line_model(p1, p2);

                // count inliers
                std::vector<size_t> candidate_inliers;
                std::vector<Point> candidate_inlier_points;
                std::vector<std::pair<Point, size_t>> inlier_data;
                for (size_t idx : remaining_indices) {
                    const Point& p = points[idx];
                    if (distance_to_line(p, candidate_line) < params.tolerance) {
                        candidate_inliers.push_back(idx);
                        candidate_inlier_points.push_back(p);
                        inlier_data.push_back({p, idx});
                    }
                }

                if (candidate_inliers.size() < inlier_thres) {
                    iter++;
                    continue;
                }

                // optimize line segment using pca
                refine_line_with_pca(candidate_line, candidate_inlier_points);

                // check line segment length
                double line_length = std::hypot(candidate_line.end.x - candidate_line.start.x,
                                                candidate_line.end.y - candidate_line.start.y);

                if (line_length < params.min_length || line_length > params.max_length) {
                    iter++;
                    continue;
                }

                // split line segments if needed
                std::vector<Line> split_lines =
                    split_line_if_needed(candidate_line, inlier_data, params.split_threshold);

                std::vector<size_t> valid_split_line_inliers;
                for (Line& l : split_lines) {
                    double split_length = std::hypot(l.end.x - l.start.x, l.end.y - l.start.y);

                    if (l.inlier_indices.size() >= params.min_inliers &&
                        split_length >= params.min_length) {
                        candidate_lines.push_back(l);
                        for (auto& idx : l.inlier_indices) {
                            valid_split_line_inliers.push_back(idx);
                        }
                    }
                }

                // remove processed inliers
                std::vector<size_t> new_remaining;
                std::set<size_t> inlier_set(valid_split_line_inliers.begin(),
                                            valid_split_line_inliers.end());
                for (size_t idx : remaining_indices) {
                    if (!inlier_set.count(idx)) {
                        new_remaining.push_back(idx);
                    }
                }
                remaining_indices = new_remaining;
            }

            for (auto& l : candidate_lines) {
                lines.push_back(l);
            }

            // reduce inlier threshold for next iteration
            inlier_thres = static_cast<size_t>(std::floor(inlier_thres * 0.9));
        }

        // sort line segments by inlier count in descending order
        std::sort(lines.begin(), lines.end(), [](const Line& a, const Line& b) {
            return a.inlier_indices.size() > b.inlier_indices.size();
        });

        return lines;
    }

   private:
    std::mt19937 rng{std::random_device{}()};  // random number generator

    // compute normalized line parameters
    Line compute_line_model(const Point& p1, const Point& p2) {
        Line line;
        line.a = p1.y - p2.y;
        line.b = p2.x - p1.x;
        line.c = p1.x * p2.y - p2.x * p1.y;

        // normalize line
        const double norm = std::hypot(line.a, line.b);
        if (norm < 1e-6) return line;  // avoid division by zero
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;

        return line;
    }

    // distance from point to line
    double distance_to_line(const Point& p, const Line& line) {
        return std::abs(line.a * p.x + line.b * p.y + line.c);
    }

    // optimize line segment endpoints using pca
    void refine_line_with_pca(Line& line, const std::vector<Point>& inliers) {
        if (inliers.size() < 2) return;

        // compute mean point
        Eigen::Vector2d mean(0, 0);
        for (const auto& p : inliers) {
            mean[0] += p.x;
            mean[1] += p.y;
        }
        mean /= inliers.size();

        // compute covariance matrix
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto& p : inliers) {
            Eigen::Vector2d v(p.x - mean[0], p.y - mean[1]);
            cov += v * v.transpose();
        }
        cov /= inliers.size();

        // eigenvalue decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
        Eigen::Vector2d direction = solver.eigenvectors().col(1);  // main direction

        // project all points onto main direction
        std::vector<double> projections;
        for (const auto& p : inliers) {
            Eigen::Vector2d v(p.x - mean[0], p.y - mean[1]);
            projections.push_back(v.dot(direction));
        }

        // find min and max of projected points
        auto minmax = std::minmax_element(projections.begin(), projections.end());
        double min_proj = *minmax.first;
        double max_proj = *minmax.second;

        // calculate endpoints
        line.start.x = mean[0] + min_proj * direction[0];
        line.start.y = mean[1] + min_proj * direction[1];
        line.end.x = mean[0] + max_proj * direction[0];
        line.end.y = mean[1] + max_proj * direction[1];
    }

    // split line if needed based on point gaps
    std::vector<Line> split_line_if_needed(const Line& line,
                                           const std::vector<std::pair<Point, size_t>>& inlier_data,
                                           double distance_threshold = 5.0) {
        Eigen::Vector2d direction(line.end.x - line.start.x, line.end.y - line.start.y);

        // handle zero vector
        if (direction.norm() < 1e-6) {
            return {line};
        }
        direction.normalize();

        // projection
        std::vector<std::tuple<double, Point, size_t>> projections;
        Eigen::Vector2d start_vec(line.start.x, line.start.y);

        for (const auto& [p, idx] : inlier_data) {
            Eigen::Vector2d vec(p.x, p.y);
            double proj = (vec - start_vec).dot(direction);
            projections.emplace_back(proj, p, idx);
        }

        // sort based on projection values
        std::sort(projections.begin(), projections.end(),
                  [](const auto& a, const auto& b) { return std::get<0>(a) < std::get<0>(b); });

        // clustering
        std::vector<std::vector<std::tuple<double, Point, size_t>>> clusters;
        std::vector<std::tuple<double, Point, size_t>> current_cluster;

        if (!projections.empty()) {
            current_cluster.push_back(projections[0]);
            for (size_t i = 1; i < projections.size(); ++i) {
                double prev_proj = std::get<0>(projections[i - 1]);
                double curr_proj = std::get<0>(projections[i]);

                if (curr_proj - prev_proj > distance_threshold) {
                    clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
                current_cluster.push_back(projections[i]);
            }
            clusters.push_back(current_cluster);
        }

        // generate new line segments
        std::vector<Line> new_lines;
        for (const auto& cluster : clusters) {
            if (cluster.size() < 2) continue;

            // extract cluster points
            std::vector<Point> cluster_points;
            std::vector<size_t> indices;
            for (const auto& [proj, p, idx] : cluster) {
                cluster_points.push_back(p);
                indices.push_back(idx);
            }

            // calculate new line segment
            Line new_line = compute_line_model(cluster_points.front(), cluster_points.back());
            refine_line_with_pca(new_line, cluster_points);
            new_line.inlier_indices = indices;
            new_lines.push_back(new_line);
        }

        return new_lines.empty() ? std::vector{line} : new_lines;
    }
};

// ===== 3d-2d projection result structure =====
struct Projected_points_result {
    std::vector<Point_3> projected_points;  // points projected onto the plane
    std::vector<size_t> original_indices;   // indices in the original point cloud
    Plane_3 plane;                          // plane used for projection

    // 2d coordinates in plane local coordinate system
    std::vector<Ransac_2d::Point> points_2d;

    // 3d-2d coordinate conversion data
    Point_3 origin;   // origin in 3d space
    Vector_3 x_axis;  // x-axis in 3d space
    Vector_3 y_axis;  // y-axis in 3d space

    // default constructor initializes vectors
    Projected_points_result() : projected_points(), original_indices(), points_2d() {}
};

// ===== 3d ransac plane detection =====

class Ransac_3d {
   public:
    struct Parameters {
        double probability;       // probability of missing the largest primitive
        size_t min_points;        // minimum number of points in shape
        double epsilon;           // maximum distance to shape
        double normal_threshold;  // maximum normal deviation
        double cluster_epsilon;   // maximum distance between adjacent points

        Parameters(double prob = 0.05, size_t min_pts = 20, double eps = 0.01,
                   double normal_thres = 0.9, double cluster_eps = 0.5)
            : probability(prob),
              min_points(min_pts),
              epsilon(eps),
              normal_threshold(normal_thres),
              cluster_epsilon(cluster_eps) {}
    };

    // detect planes from point cloud with normals
    static std::vector<Plane_result> detect_planes(const std::vector<Point_3>& points,
                                                   const std::vector<Vector_3>& normals,
                                                   const Parameters& params = Parameters()) {
        // create point-normal pair vector
        Pwn_vector pwn_vector;

        for (size_t i = 0; i < points.size(); ++i) {
            pwn_vector.emplace_back(points[i], normals[i]);
        }

        // configure ransac
        Efficient_ransac ransac;
        ransac.set_input(pwn_vector);
        ransac.add_shape_factory<Plane_shape>();

        // set parameters
        Efficient_ransac::Parameters ransac_params;
        ransac_params.probability = params.probability;
        ransac_params.min_points = params.min_points;
        ransac_params.epsilon = params.epsilon;
        ransac_params.normal_threshold = params.normal_threshold;
        ransac_params.cluster_epsilon = params.cluster_epsilon;

        // detect shapes
        ransac.detect(ransac_params);

        // extract results
        std::vector<Plane_result> results;
        for (const auto& shape : ransac.shapes()) {
            const Plane_shape* plane_shape = dynamic_cast<const Plane_shape*>(shape.get());
            if (plane_shape) {
                Plane_result result;
                result.plane = *plane_shape;  // use implicit conversion from plane_shape to plane_3

                // get inliers and store corresponding point-normal pairs
                auto ransac_inliers = plane_shape->indices_of_assigned_points();
                result.inlier_indices =
                    std::vector<std::size_t>(ransac_inliers.begin(), ransac_inliers.end());

                // store point-normal pairs for each plane
                for (const auto& idx : result.inlier_indices) {
                    if (idx < pwn_vector.size()) {
                        result.points_with_normals.push_back(pwn_vector[idx]);
                    }
                }

                results.push_back(result);
            }
        }

        // sort results by inlier count in descending order
        std::sort(results.begin(), results.end(), [](const Plane_result& a, const Plane_result& b) {
            return a.inlier_indices.size() > b.inlier_indices.size();
        });

        return results;
    }

    // project points to plane keeping z-coordinate constant
    static Projected_points_result project_points_to_plane(const std::vector<Point_3>& all_points,
                                                           const Plane_result& plane_result,
                                                           double distance_threshold = 0.15) {
        Projected_points_result result;
        result.plane = plane_result.plane;
        const Plane_3& plane = plane_result.plane;

        // get plane normal vector
        Vector_3 plane_normal = plane.orthogonal_vector();
        plane_normal = plane_normal / std::sqrt(plane_normal.squared_length());

        // check if plane is near horizontal or vertical
        bool is_near_horizontal = std::abs(plane_normal.z()) > 0.95;  // near horizontal
        bool is_near_vertical = std::abs(plane_normal.z()) < 0.05;   // near vertical

        // get orthogonal basis for the plane using built-in methods
        Vector_3 x_axis = plane.base1();
        Vector_3 y_axis = plane.base2();
        
        // normalize the vectors
        x_axis = x_axis / std::sqrt(x_axis.squared_length());
        y_axis = y_axis / std::sqrt(y_axis.squared_length());

        // get a point on the plane as origin
        Point_3 origin = plane.point();

        // store coordinate system for later use
        result.origin = origin;
        result.x_axis = x_axis;
        result.y_axis = y_axis;

        // get z-value range of points on the plane
        double min_z = std::numeric_limits<double>::infinity();
        double max_z = -std::numeric_limits<double>::infinity();
        for (const auto& pwn : plane_result.points_with_normals) {
            min_z = std::min(min_z, pwn.first.z());
            max_z = std::max(max_z, pwn.first.z());
        }

        // process each point
        for (size_t i = 0; i < all_points.size(); ++i) {
            const Point_3& point = all_points[i];
            
            // check if z-value is within range
            if (point.z() < min_z || point.z() > max_z) {
                continue;
            }

            // calculate projected point
            Point_3 projected_point;
            if (is_near_horizontal || is_near_vertical) {
                projected_point = plane.projection(point);
            } else {
                // for other cases, project point onto the plane but keep z-coordinate constant
                Vector_3 v = point - origin;
                double t = -(v * plane_normal) / (plane_normal * plane_normal);
                projected_point = Point_3(
                    point.x() + t * plane_normal.x(),
                    point.y() + t * plane_normal.y(),
                    point.z() + t * plane_normal.z()
                );
            }

            // calculate euclidean distance between original point and projected point
            double euclidean_distance = std::sqrt(
                std::pow(point.x() - projected_point.x(), 2) +
                std::pow(point.y() - projected_point.y(), 2) +
                std::pow(point.z() - projected_point.z(), 2)
            );
            
            if (euclidean_distance > distance_threshold) {
                continue;
            }

            // calculate 2d coordinates
            auto p2d = plane.to_2d(projected_point);
            Ransac_2d::Point point_2d;
            point_2d.x = p2d.x();
            point_2d.y = p2d.y();

            result.projected_points.push_back(projected_point);
            result.points_2d.push_back(point_2d);
            result.original_indices.push_back(i);
        }

        return result;
    }

    // convert 2d point in plane coordinate system back to 3d
    static Point_3 convert_2d_to_3d(const Ransac_2d::Point& point_2d,
                                    const Projected_points_result& projection_data) {
        // use stored coordinate system to convert back to 3d
        Point_3 point_3d = projection_data.origin + projection_data.x_axis * point_2d.x +
                           projection_data.y_axis * point_2d.y;

        return point_3d;
    }

    // convert 2d line in plane coordinate system back to 3d
    static std::pair<Point_3, Point_3> convert_line_2d_to_3d(
        const Ransac_2d::Line& line_2d, const Projected_points_result& projection_data) {
        // convert line start and end points to 3d
        Point_3 start_3d = convert_2d_to_3d(line_2d.start, projection_data);
        Point_3 end_3d = convert_2d_to_3d(line_2d.end, projection_data);

        return std::make_pair(start_3d, end_3d);
    }
};

// project point cloud to principal plane using PCA
static Projected_points_result project_to_principal_plane(
    const std::vector<Point_3>& points,
    double distance_threshold = 0.15) {
    Projected_points_result result;
    
    if (points.empty()) {
        return result;
    }

    // compute mean point
    Point_3 mean(0, 0, 0);
    for (const auto& p : points) {
        mean = Point_3(mean.x() + p.x(), mean.y() + p.y(), mean.z() + p.z());
    }
    mean = Point_3(mean.x() / points.size(), mean.y() / points.size(), mean.z() / points.size());

    // compute covariance matrix
    Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
    for (const auto& p : points) {
        Eigen::Vector3d v(p.x() - mean.x(), p.y() - mean.y(), p.z() - mean.z());
        cov += v * v.transpose();
    }
    cov /= points.size();

    // eigenvalue decomposition
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(cov);
    
    // 获取特征值和特征向量
    Eigen::Vector3d eigenvalues = solver.eigenvalues();
    Eigen::Matrix3d eigenvectors = solver.eigenvectors();
    
    // 按特征值大小排序
    std::vector<size_t> sorted_indices(3);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::sort(sorted_indices.begin(), sorted_indices.end(),
              [&eigenvalues](size_t i1, size_t i2) { return eigenvalues[i1] > eigenvalues[i2]; });
    
    // 使用前两个最大特征值对应的特征向量作为投影平面的基向量
    Eigen::Vector3d x_eigen = eigenvectors.col(sorted_indices[0]);  // first principal component
    Eigen::Vector3d y_eigen = eigenvectors.col(sorted_indices[1]);  // second principal component
    Eigen::Vector3d normal = eigenvectors.col(sorted_indices[2]);   // third principal component (depth)
    
    // 确保是右手坐标系
    Eigen::Vector3d cross_product = x_eigen.cross(y_eigen);
    if (cross_product.dot(normal) < 0) {
        y_eigen = -y_eigen;
    }
    
    // convert Eigen vectors to CGAL vectors
    Vector_3 x_axis(x_eigen.x(), x_eigen.y(), x_eigen.z());
    Vector_3 y_axis(y_eigen.x(), y_eigen.y(), y_eigen.z());
    Vector_3 normal_vector(normal.x(), normal.y(), normal.z());
    
    // normalize the vectors
    x_axis = x_axis / std::sqrt(x_axis.squared_length());
    y_axis = y_axis / std::sqrt(y_axis.squared_length());
    normal_vector = normal_vector / std::sqrt(normal_vector.squared_length());
    
    // create plane using normal and mean point
    result.plane = Plane_3(Point_3(mean.x(), mean.y(), mean.z()), normal_vector);

    // store coordinate system
    result.origin = Point_3(mean.x(), mean.y(), mean.z());
    result.x_axis = x_axis;
    result.y_axis = y_axis;

    // 计算所有点的深度值
    std::vector<double> depth_values;
    for (const auto& p : points) {
        Eigen::Vector3d v(p.x() - mean.x(), p.y() - mean.y(), p.z() - mean.z());
        depth_values.push_back(v.dot(normal));
    }
    
    // 计算深度阈值
    double max_depth = *std::max_element(depth_values.begin(), depth_values.end());
    double depth_thres = distance_threshold * max_depth;

    // project points to plane
    for (size_t i = 0; i < points.size(); ++i) {
        const Point_3& point = points[i];
        
        // 根据深度值过滤点
        if (std::abs(depth_values[i]) > depth_thres) {
            continue;
        }
        
        // 计算点相对于中心点的偏移
        Eigen::Vector3d v(point.x() - mean.x(), point.y() - mean.y(), point.z() - mean.z());
        
        // 直接投影到主方向
        double x_proj = v.dot(x_eigen);
        double y_proj = v.dot(y_eigen);
        
        // 计算投影点
        Point_3 projected_point(
            mean.x() + x_proj * x_eigen.x() + y_proj * y_eigen.x(),
            mean.y() + x_proj * x_eigen.y() + y_proj * y_eigen.y(),
            mean.z() + x_proj * x_eigen.z() + y_proj * y_eigen.z()
        );

        // 计算2D坐标
        Ransac_2d::Point point_2d;
        point_2d.x = x_proj;
        point_2d.y = y_proj;

        result.projected_points.push_back(projected_point);
        result.points_2d.push_back(point_2d);
        result.original_indices.push_back(i);
    }

    return result;
}
}  // namespace custom_ransac

#endif  // CUSTOM_RANSAC_H