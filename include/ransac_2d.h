#ifndef RANSAC_2D_H
#define RANSAC_2D_H

#include <stdio.h>
#include <stdlib.h>

#include <algorithm>
#include <cmath>
#include <ctime>
#include <vector>


class Ransac_2d {
   public:
    struct Point {
        double x;
        double y;
    };

    struct Line {
        double slope;
        double intercept;
        Point start;
        Point end;
        std::vector<Point> inliers;
        std::vector<size_t> inliers_indices;
    };

   protected:
    double distance(Point p1, Point p2) {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return sqrt(dx * dx + dy * dy);
    }

    Line detect_single_line(const std::vector<Point>& points, const std::vector<size_t>& indices,
                            const size_t max_iterations, const size_t min_points,
                            const size_t min_inliers, const size_t tolerance) {
        std::srand(std::time(nullptr));
        Line bestline;
        int bestInlierNums = 0;

        for (int i = 0; i < max_iterations; ++i) {
            // pick two points randomly
            size_t index1 = std::rand() % indices.size();
            size_t index2 = std::rand() % indices.size();
            while (index1 == index2) {
                index2 = std::rand() % indices.size();
            }
            const Point& p1 = points[index1];
            const Point& p2 = points[index2];

            // calculate line parameters
            double slope = (p2.y - p1.y) / (p2.x - p1.x + 0.0001);
            double intercept = p1.y - slope * p1.x;

            // count inliers
            std::vector<Point> inliers;
            std::vector<size_t> inliers_indices;
            for (size_t idx = 0; idx < points.size(); ++idx) {
                const Point& p = points[idx];
                double d = std::abs(p.y - slope * p.x - intercept) / std::sqrt(1 + slope * slope);
                if (d < tolerance) {
                    inliers.push_back(p);
                    inliers_indices.push_back(indices[idx]);
                }
            }

            // update best line
            if (inliers.size() > bestInlierNums && inliers.size() >= min_inliers) {
                bestInlierNums = inliers.size();
                bestline.slope = slope;
                bestline.intercept = intercept;
                bestline.inliers = inliers;
                bestline.inliers_indices = inliers_indices;
            }
        }

        // calcualte end points of the best line
        if (!bestline.inliers.empty()) {
            auto x_minmax =
                std::minmax_element(bestline.inliers.begin(), bestline.inliers.end(),
                                    [](const Point& p1, const Point& p2) { return p1.x < p2.x; });
            bestline.start = *x_minmax.first;
            bestline.end = *x_minmax.second;
        }

        return bestline;
    }

   public:
    std::vector<Line> detect(const std::vector<Point>& points, const size_t max_iterations = 100,
                             const size_t min_points = 2, const size_t min_inliers = 10,
                             const double tolerance = 0.1) {
        std::vector<Line> detected_lines;
        std::vector<size_t> remaining_indices(points.size());
        std::iota(remaining_indices.begin(), remaining_indices.end(), 0);
        size_t iter = 0;
        while (remaining_indices.size() >= min_inliers && iter++ < max_iterations) {
            std::vector<Point> remaining_points;
            for (size_t idx : remaining_indices) {
                remaining_points.push_back(points[idx]);
            }
            Line line = detect_single_line(remaining_points, remaining_indices, max_iterations,
                                           min_points, min_inliers, tolerance);

            // Stop if no sufficient inliers are found
            std::vector<size_t> inliers_indices = line.inliers_indices;
            if (inliers_indices.size() < min_inliers) {
                continue;
            }

            detected_lines.push_back(line);

            // Remove inliers' indices from the remaining indices
            std::vector<size_t> new_remaining_indices;
            for (size_t idx : remaining_indices) {
                if (std::find(inliers_indices.begin(), inliers_indices.end(), idx) ==
                    inliers_indices.end()) {
                    new_remaining_indices.push_back(idx);
                }
            }
            remaining_indices = new_remaining_indices;
        }

        return detected_lines;
    }
};

#endif  // RANSAC_2D_H