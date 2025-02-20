import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Line:
    a: float  # Line equation: a*x + b*y + c = 0
    b: float
    c: float
    start: Point  # Line endpoints (obtained through PCA or fitting)
    end: Point
    length: float
    inlier_indices: List[int]


class HoughTransform2D:
    def __init__(self, num_theta: int = 180, rho_resolution: float = 0.1):
        """
        Initialize Hough transform parameters.
        :param num_theta: Number of discrete angles in [0, π) interval.
        :param rho_resolution: Resolution of the ρ axis.
        """
        self.num_theta = num_theta
        self.rho_resolution = rho_resolution
        self.thetas = np.linspace(0, np.pi, num_theta, endpoint=False)

    def _refine_line_with_pca(self, line: Line, inliers: List[Point]) -> None:
        """
        Refit the line using PCA on inlier points, update endpoints and length.
        """
        if len(inliers) < 2:
            return
        pts = np.array([[p.x, p.y] for p in inliers])
        mean = np.mean(pts, axis=0)
        cov = np.cov(pts.T)
        eigenvalues, eigenvectors = np.linalg.eig(cov)
        max_idx = np.argmax(eigenvalues)
        direction = eigenvectors[:, max_idx]
        projections = (pts - mean).dot(direction)
        min_proj = np.min(projections)
        max_proj = np.max(projections)
        start_point = mean + min_proj * direction
        end_point = mean + max_proj * direction
        line.start = Point(x=float(start_point[0]), y=float(start_point[1]))
        line.end = Point(x=float(end_point[0]), y=float(end_point[1]))
        line.length = np.hypot(line.end.x - line.start.x, line.end.y - line.start.y)

    def _compute_line_model(self, p1: Point, p2: Point) -> Line:
        """
        Construct line model from two points and normalize parameters.
        """
        a = p1.y - p2.y
        b = p2.x - p1.x
        c = p1.x * p2.y - p2.x * p1.y
        norm = np.hypot(a, b)
        if norm < 1e-6:
            return Line(a=a, b=b, c=c, start=p1, end=p2, length=0.0, inlier_indices=[])
        a, b, c = a / norm, b / norm, c / norm
        return Line(
            a=a,
            b=b,
            c=c,
            start=p1,
            end=p2,
            length=np.hypot(p2.x - p1.x, p2.y - p1.y),
            inlier_indices=[],
        )

    def _split_line_if_needed(
        self,
        line: Line,
        inlier_data: List[Tuple[Point, int]],
        distance_threshold: float = 5.0,
    ) -> List[Line]:
        """
        Cluster inlier points based on their projections along the line direction.
        Split into multiple lines if adjacent projections exceed distance_threshold.
        :param line: Detected line model.
        :param inlier_data: List of (Point, index) tuples.
        :param distance_threshold: Clustering distance threshold.
        :return: List of split lines, returns [line] if no split needed.
        """
        # Calculate line direction vector (from line.start to line.end)
        direction = np.array([line.end.x - line.start.x, line.end.y - line.start.y])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return [line]
        direction = direction / norm

        # Project each inlier to the line direction
        projections = []
        for p, idx in inlier_data:
            vec = np.array([p.x - line.start.x, p.y - line.start.y])
            proj = np.dot(vec, direction)
            projections.append((proj, p, idx))
        # Sort by projection value
        projections.sort(key=lambda x: x[0])

        # Cluster based on projection gaps
        clusters = []
        current_cluster = [projections[0]]
        for i in range(1, len(projections)):
            if projections[i][0] - projections[i - 1][0] > distance_threshold:
                clusters.append(current_cluster)
                current_cluster = []
            current_cluster.append(projections[i])
        clusters.append(current_cluster)

        if len(clusters) == 1:
            return [line]

        new_lines = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue
            cluster_points = [item[1] for item in cluster]
            cluster_indices = [item[2] for item in cluster]
            # Create initial line model using first and last points in cluster
            new_line = self._compute_line_model(cluster_points[0], cluster_points[-1])
            self._refine_line_with_pca(new_line, cluster_points)
            new_line.inlier_indices = cluster_indices
            new_lines.append(new_line)
        return new_lines

    def detect(
        self,
        points: List[Point],
        vote_threshold: int = 10,
        tolerance: float = 0.1,
        max_lines: int = 100,
        split_distance_threshold: float = 5.0,
    ) -> List[Line]:
        """
        Iterative Hough Transform line detection:
        1. Vote on remaining points and select highest vote bin
        2. Use voting points in bin as inliers to build initial line model
        3. Optimize candidate line with PCA
        4. Split inliers using _split_line_if_needed (similar to ransac_2d)
        5. Remove inliers and continue detection
        
        :param points: List of 2D points
        :param vote_threshold: Minimum votes for valid line candidate
        :param tolerance: Distance tolerance (used with vote threshold)
        :param max_lines: Maximum number of lines to detect
        :param split_distance_threshold: Max allowed gap between inlier projections
        :return: List of detected lines
        """
        detected_lines = []
        remaining_points = points.copy()

        while (
            len(remaining_points) >= vote_threshold and len(detected_lines) < max_lines
        ):
            # Calculate ρ range for remaining points
            xs = np.array([p.x for p in remaining_points])
            ys = np.array([p.y for p in remaining_points])
            diag_len = np.hypot(np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))
            rho_min = -diag_len
            rho_max = diag_len
            rhos = np.arange(rho_min, rho_max, self.rho_resolution)
            num_rhos = len(rhos)

            # Initialize accumulator and vote tracking
            accumulator = np.zeros((num_rhos, len(self.thetas)), dtype=int)
            votes = [[[] for _ in range(len(self.thetas))] for _ in range(num_rhos)]
            cos_t = np.cos(self.thetas)
            sin_t = np.sin(self.thetas)

            # Voting process
            for idx, p in enumerate(remaining_points):
                computed_rhos = p.x * cos_t + p.y * sin_t
                bin_indices = np.round(
                    (computed_rhos - rho_min) / self.rho_resolution
                ).astype(int)
                for j, i in enumerate(bin_indices):
                    if 0 <= i < num_rhos:
                        accumulator[i, j] += 1
                        votes[i][j].append(idx)

            max_vote = np.max(accumulator)
            print(f"max vote: {max_vote}")
            if max_vote < vote_threshold:
                break

            i_max, j_max = np.unravel_index(np.argmax(accumulator), accumulator.shape)
            # Get candidate line parameters
            rho_val = rhos[i_max]
            theta_val = self.thetas[j_max]
            a = np.cos(theta_val)
            b = np.sin(theta_val)
            c = -rho_val

            # Get inliers from voting bin
            inlier_indices = votes[i_max][j_max]
            inlier_points = [remaining_points[idx] for idx in inlier_indices]

            # Build and refine line model
            candidate_line = self._compute_line_model(inlier_points[0], inlier_points[-1])
            self._refine_line_with_pca(candidate_line, inlier_points)

            # Prepare data for splitting
            inlier_data = [(remaining_points[idx], idx) for idx in inlier_indices]
            split_lines = self._split_line_if_needed(
                candidate_line, inlier_data, distance_threshold=split_distance_threshold
            )
            for l in split_lines:
                if len(l.inlier_indices) >= vote_threshold:
                    detected_lines.append(l)

            # Remove processed inliers
            remaining_points = [
                p for idx, p in enumerate(remaining_points) if idx not in set(inlier_indices)
            ]

        print(f"Total detected lines: {len(detected_lines)}")
        detected_lines = sorted(detected_lines, key=lambda l: len(l.inlier_indices), reverse=True)
        return detected_lines