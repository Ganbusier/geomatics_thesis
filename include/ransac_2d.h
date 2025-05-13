#ifndef RANSAC_2D_H
#define RANSAC_2D_H

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <ctime>
#include <numeric>
#include <random>
#include <set>
#include <vector>

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

   private:
    std::mt19937 rng{std::random_device{}()};  // random number generator

    // compute normalized line parameters
    Line computeLineModel(const Point& p1, const Point& p2) {
        Line line;
        line.a = p1.y - p2.y;
        line.b = p2.x - p1.x;
        line.c = p1.x * p2.y - p2.x * p1.y;

        // normalize line
        const double norm = std::hypot(line.a, line.b);
        if (norm < 1e-6) return line;  // avoid divided by zero
        line.a /= norm;
        line.b /= norm;
        line.c /= norm;

        return line;
    }

    // point to line distance
    double distanceToLine(const Point& p, const Line& line) {
        return std::abs(line.a * p.x + line.b * p.y + line.c);
    }

    // use PCA to optimize the two end points of a line
    void refineLineWithPCA(Line& line, const std::vector<Point>& inliers) {
        if (inliers.size() < 2) return;

        // mean point
        Eigen::Vector2d mean(0, 0);
        for (const auto& p : inliers) {
            mean[0] += p.x;
            mean[1] += p.y;
        }
        mean /= inliers.size();

        // covariance matrix
        Eigen::Matrix2d cov = Eigen::Matrix2d::Zero();
        for (const auto& p : inliers) {
            Eigen::Vector2d v(p.x - mean[0], p.y - mean[1]);
            cov += v * v.transpose();
        }
        cov /= inliers.size();

        // eigen decomposition
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(cov);
        Eigen::Vector2d direction = solver.eigenvectors().col(1);  // main direction

        // project all points to the main direction
        std::vector<double> projections;
        for (const auto& p : inliers) {
            Eigen::Vector2d v(p.x - mean[0], p.y - mean[1]);
            projections.push_back(v.dot(direction));
        }

        // find min and max of projected points
        auto minmax = std::minmax_element(projections.begin(), projections.end());
        double min_proj = *minmax.first;
        double max_proj = *minmax.second;

        // calculate end points
        line.start.x = mean[0] + min_proj * direction[0];
        line.start.y = mean[1] + min_proj * direction[1];
        line.end.x = mean[0] + max_proj * direction[0];
        line.end.y = mean[1] + max_proj * direction[1];
    }

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

        // generate new lines
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

            // calculate new lines
            Line new_line = computeLineModel(cluster_points.front(), cluster_points.back());
            refineLineWithPCA(new_line, cluster_points);
            new_line.inlier_indices = indices;
            new_lines.push_back(new_line);
        }

        return new_lines.empty() ? std::vector{line} : new_lines;
    }

   public:
    std::vector<Line> detect(const std::vector<Point>& points, size_t max_iterations = 1000,
                             size_t min_inliers = 5, double tolerance = 0.1,
                             double split_distance_thres = 1.0) {
        std::vector<Line> lines;
        std::vector<size_t> all_remaining_indices(points.size());
        std::iota(all_remaining_indices.begin(), all_remaining_indices.end(), 0);
        size_t inlier_thres = std::floor(points.size() * 0.1);
        size_t min_model_samples = 2;
        if (inlier_thres <= min_inliers) return lines;

        std::vector<size_t> remaining_indices = all_remaining_indices;
        while (inlier_thres >= min_inliers) {
            std::cout << " Remaining indices: " << remaining_indices.size();
            std::cout << " Current inliers threshold: " << inlier_thres << std::endl;
            std::vector<Line> candidate_lines;

            size_t iter = 0;
            size_t iter2 = 0;
            while (iter < max_iterations && 
                   iter2 < 10000 &&
                   remaining_indices.size() > inlier_thres && 
                   inlier_thres > 4) {
                // sample two points randomly
                std::vector<size_t> sample_indices(2);
                std::sample(remaining_indices.begin(), remaining_indices.end(),
                            sample_indices.begin(), 2, rng);
                const Point& p1 = points[sample_indices[0]];
                const Point& p2 = points[sample_indices[1]];

                // pass if points are overlapped or too far
                if (std::hypot(p1.x - p2.x, p1.y - p2.y) < 1e-6) { iter2++; continue; }
                if (std::hypot(p1.x - p2.x, p1.y - p2.y) > 0.5) { iter2++; continue; }

                // compute candidate line
                Line candidate_line = computeLineModel(p1, p2);

                // count inliers
                std::vector<size_t> candidate_inliers;
                std::vector<Point> candidate_inlier_points;
                std::vector<std::pair<Point, size_t>> inlier_data;
                for (size_t idx : remaining_indices) {
                    const Point& p = points[idx];
                    if (distanceToLine(p, candidate_line) < tolerance) {
                        candidate_inliers.push_back(idx);
                        candidate_inlier_points.push_back(p);
                        inlier_data.push_back({p, idx});
                    }
                }

                if (candidate_inliers.size() < inlier_thres) {
                    iter++;
                    continue;
                }
                refineLineWithPCA(candidate_line, candidate_inlier_points);

                std::vector<Line> split_lines =
                    split_line_if_needed(candidate_line, inlier_data, split_distance_thres);

                std::vector<size_t> valid_split_line_inliers;
                for (const Line& l : split_lines) {
                    if (l.inlier_indices.size() >= min_inliers) {
                        candidate_lines.push_back(l);
                        for (auto& idx : l.inlier_indices) {
                            valid_split_line_inliers.push_back(idx);
                        }
                    }
                }

                // remove inliers
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
            inlier_thres = std::floor(inlier_thres * 0.9);
        }

        return lines;
    }
};

#endif  // RANSAC_2D_H