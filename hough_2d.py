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
        current_vote_threshold = np.floor(len(points) * 0.1)

        while current_vote_threshold >= vote_threshold:
            print(f"Current vote threshold: {current_vote_threshold}, "
                  f"Remaining indices: {len(remaining_points)}")
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
            if max_vote < vote_threshold:
                break

            candidates = np.argwhere(accumulator >= current_vote_threshold)
            sorted_candidates = sorted(candidates, key=lambda x: accumulator[x[0], x[1]], reverse=True)

            # 去重处理（合并相近参数候选）
            unique_candidates = []
            tolerance_rho = 5  # rho参数容差（单位：像素）
            tolerance_theta = np.deg2rad(2)  # theta参数容差（单位：弧度）

            all_inlier_indices = []
            for (i, j) in sorted_candidates:
                rho_val = rhos[i]
                theta_val = self.thetas[j]
                # 检查是否与已选候选参数相近
                is_unique = True
                for (u_rho, u_theta) in unique_candidates:
                    if (abs(rho_val - u_rho) < tolerance_rho 
                        and abs(theta_val - u_theta) < tolerance_theta):
                        is_unique = False
                        break
                if is_unique:
                    unique_candidates.append((rho_val, theta_val))

            # 遍历所有唯一候选
            for rho_val, theta_val in unique_candidates:
                # 获取该候选的inlier索引
                i = np.where(rhos == rho_val)[0][0]
                j = np.where(self.thetas == theta_val)[0][0]
                inlier_indices = votes[i][j]

                # 跳过已处理的inliers（避免重复）
                if len(inlier_indices) == 0:
                    continue

                # 后续处理（与原始代码一致）
                inlier_points = [remaining_points[idx] for idx in inlier_indices]
                candidate_line = self._compute_line_model(inlier_points[0], inlier_points[-1])
                self._refine_line_with_pca(candidate_line, inlier_points)

                # 拆分线段（若需要）
                inlier_data = [(remaining_points[idx], idx) for idx in inlier_indices]
                split_lines = self._split_line_if_needed(
                    candidate_line, inlier_data, distance_threshold=split_distance_threshold
                )

                # 添加有效线段
                for l in split_lines:
                    if len(l.inlier_indices) >= vote_threshold:
                        detected_lines.append(l)
                        all_inlier_indices.extend(l.inlier_indices)

            # 标记已处理的inliers
            remaining_points = [p for idx, p in enumerate(remaining_points) if idx not in set(all_inlier_indices)]
            current_vote_threshold = np.floor(current_vote_threshold * 0.8)

        print(f"Total detected lines: {len(detected_lines)}")
        detected_lines = sorted(detected_lines, key=lambda l: len(l.inlier_indices), reverse=True)
        return detected_lines