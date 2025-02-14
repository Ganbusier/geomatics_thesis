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

   public:
    std::vector<Line> detect(const std::vector<Point>& points, size_t max_iterations = 1000,
                             size_t min_inliers = 2, double tolerance = 0.1) {
        std::vector<Line> lines;
        std::vector<size_t> remaining_indices(points.size());
        std::iota(remaining_indices.begin(), remaining_indices.end(), 0);

        while (remaining_indices.size() >= min_inliers) {
            Line best_line;
            size_t best_inliers = 0;

            for (size_t iter = 0; iter < max_iterations; ++iter) {
                // sample two points randomly
                std::vector<size_t> shuffled_indices = remaining_indices;
                std::shuffle(shuffled_indices.begin(), shuffled_indices.end(), rng);
                const Point& p1 = points[shuffled_indices[0]];
                const Point& p2 = points[shuffled_indices[1]];

                // pass if points are overlapped or too far
                if (std::hypot(p1.x - p2.x, p1.y - p2.y) < 1e-6) continue;
                if (std::hypot(p1.x - p2.x, p1.y - p2.y) > 2.0) continue;

                // compute candidate line
                Line candidate_line = computeLineModel(p1, p2);

                // count inliers
                std::vector<size_t> candidate_inliers;
                for (size_t idx : remaining_indices) {
                    const Point& p = points[idx];
                    if (distanceToLine(p, candidate_line) < tolerance) {
                        candidate_inliers.push_back(idx);
                    }
                }

                // update the best line
                if (candidate_inliers.size() > best_inliers) {
                    best_inliers = candidate_inliers.size();
                    best_line = candidate_line;
                    best_line.inlier_indices = candidate_inliers;
                }
            }

            if (best_inliers >= min_inliers) {
                // use PCA to optimize end points of a line
                std::vector<Point> inlier_points;
                for (auto idx : best_line.inlier_indices) {
                    inlier_points.push_back(points[idx]);
                }
                refineLineWithPCA(best_line, inlier_points);

                lines.push_back(best_line);

                // remove inliers
                std::vector<size_t> new_remaining;
                std::set<size_t> inlier_set(best_line.inlier_indices.begin(),
                                            best_line.inlier_indices.end());
                for (size_t idx : remaining_indices) {
                    if (!inlier_set.count(idx)) {
                        new_remaining.push_back(idx);
                    }
                }
                remaining_indices = new_remaining;
            } else {
                break;  // if not enough inliers, terminate
            }
        }

        return lines;
    }
};

#endif  // RANSAC_2D_H