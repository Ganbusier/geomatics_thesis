import numpy as np
import random
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Line:
    a: float  # ax + by + c = 0
    b: float
    c: float
    start: Point
    end: Point
    length: float
    inlier_indices: List[int]


class Ransac2D:
    def __init__(self):
        self.rng = random.Random()

    def _compute_line_model(self, p1: Point, p2: Point) -> Line:
        a = p1.y - p2.y
        b = p2.x - p1.x
        c = p1.x * p2.y - p2.x * p1.y

        norm = np.hypot(a, b)
        if norm < 1e-6:
            return Line(a=a, b=b, c=c, start=p1, end=p2, inlier_indices=[])

        a_norm = a / norm
        b_norm = b / norm
        c_norm = c / norm

        length = np.hypot(p2.x - p1.x, p2.y - p1.y)

        return Line(
            a=a_norm,
            b=b_norm,
            c=c_norm,
            start=p1,
            end=p2,
            length=length,
            inlier_indices=[],
        )

    def _distance_to_line(self, p: Point, line: Line) -> float:
        return abs(line.a * p.x + line.b * p.y + line.c)

    def _refine_line_with_pca(self, line: Line, inliers: List[Point]) -> None:
        if len(inliers) < 2:
            return

        points = np.array([(p.x, p.y) for p in inliers])
        mean = np.mean(points, axis=0)
        cov = np.cov(points.T, ddof=0)

        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        max_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, max_idx]

        centered = points - mean
        projections = centered.dot(direction)

        min_proj = np.min(projections)
        max_proj = np.max(projections)

        start_point = mean + min_proj * direction
        end_point = mean + max_proj * direction

        line.start = Point(x=start_point[0], y=start_point[1])
        line.end = Point(x=end_point[0], y=end_point[1])

    def _split_line_if_needed(
        self,
        line: Line,
        inlier_data: List[Tuple[Point, int]],
        distance_threshold: float = 5.0,
    ) -> List[Line]:
        """
        project inliers, if there is a pair of points that has a projected distance > distance_threshold,
        split the input line into multiple lines.
        inlier_data: each element is (Point, index).
        return a new list of Line objects, each line represents an inlier-cluster.
        """
        # use current line end points to calculate line direction
        direction = np.array([line.end.x - line.start.x, line.end.y - line.start.y])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return [line]
        direction = direction / norm

        # project inliers to the line direction
        projections = []
        for p, idx in inlier_data:
            vec = np.array([p.x, p.y]) - np.array([line.start.x, line.start.y])
            proj = np.dot(vec, direction)
            projections.append((proj, p, idx))

        # sorted by projection values
        projections.sort(key=lambda x: x[0])

        # clustering based on projection values
        clusters = []
        current_cluster = [projections[0]]
        for i in range(1, len(projections)):
            prev_proj = projections[i - 1][0]
            cur_proj = projections[i][0]
            if cur_proj - prev_proj > distance_threshold:
                clusters.append(current_cluster)
                current_cluster = []
            current_cluster.append(projections[i])
        clusters.append(current_cluster)

        # if only one cluster, no need to split
        if len(clusters) == 1:
            return [line]

        # for each cluster, use PCA to recalculate line model
        new_lines = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            # extract inliers and their indices for each cluster
            cluster_points = [p for (_, p, _) in cluster]
            cluster_indices = [idx for (_, _, idx) in cluster]
            # use the first and the last points to construct initial line model and refine with PCA
            new_line = self._compute_line_model(cluster_points[0], cluster_points[-1])
            self._refine_line_with_pca(new_line, cluster_points)
            new_line.inlier_indices = cluster_indices
            new_lines.append(new_line)
        return new_lines

    def _avg_adjacent_inlier_distance(
        self, line: Line, inlier_data: List[Tuple[Point, int]]
    ) -> float:
        """
        calculate average adjacent-inlier-distance after sort the inliers based on the line direction.
        """
        direction = np.array([line.end.x - line.start.x, line.end.y - line.start.y])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return 0.0
        direction = direction / norm

        # project inliers to the line direction
        projections = []
        for p, idx in inlier_data:
            vec = np.array([p.x, p.y]) - np.array([line.start.x, line.start.y])
            proj = np.dot(vec, direction)
            projections.append(proj)
        projections.sort()
        if len(projections) < 2:
            return 0.0
        # calculate diff between adjacent inliers
        diffs = [
            projections[i + 1] - projections[i] for i in range(len(projections) - 1)
        ]
        avg_diff = sum(diffs) / len(diffs)
        return avg_diff

    def detect(
        self,
        points: List[Point],
        max_iterations: int = 1000,
        min_inliers: int = 2,
        tolerance: float = 0.1,
        split_distance_threshold: int = 5.0,
    ) -> List[Line]:
        lines = []
        remaining_indices = list(range(len(points)))

        while len(remaining_indices) >= min_inliers:
            print(len(remaining_indices))
            best_line = None
            best_inliers_count = 0
            best_inliers_indices = []

            for _ in range(max_iterations):
                idx1, idx2 = random.sample(remaining_indices, k=2)
                p1, p2 = points[idx1], points[idx2]

                dx, dy = p1.x - p2.x, p1.y - p2.y
                distance = np.hypot(dx, dy)
                if distance < 1e-6:
                    continue

                # construct initial line model
                candidate_line = self._compute_line_model(p1, p2)
                # detect inliers
                candidate_inliers = [
                    idx
                    for idx in remaining_indices
                    if self._distance_to_line(points[idx], candidate_line) < tolerance
                ]
                candidate_line.inlier_indices = candidate_inliers
                inlier_data = [(points[idx], idx) for idx in candidate_inliers]

                # update best line and best inliers
                if len(candidate_inliers) > best_inliers_count:
                    best_inliers_count = len(candidate_inliers)
                    best_line = candidate_line
                    best_inliers_indices = candidate_inliers

            # validate the final best line of an iteration
            if best_line and best_inliers_count >= min_inliers:
                inlier_points = [points[idx] for idx in best_inliers_indices]
                self._refine_line_with_pca(best_line, inlier_points)
                inlier_data = [(points[idx], idx) for idx in best_inliers_indices]
                split_lines = self._split_line_if_needed(
                    best_line, inlier_data, distance_threshold=split_distance_threshold
                )
                for l in split_lines:
                    if len(l.inlier_indices) > min_inliers:
                        lines.append(l)
                remaining_indices = [
                    idx
                    for idx in remaining_indices
                    if idx not in set(best_inliers_indices)
                ]
            else:
                break
        print(len(lines))
        lines = sorted(lines, key=lambda l: len(l.inlier_indices), reverse=True)
        return lines
