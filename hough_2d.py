import numpy as np
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Point:
    x: float
    y: float


@dataclass
class Line:
    a: float  # 直线方程：a*x + b*y + c = 0
    b: float
    c: float
    start: Point  # 直线端点（通过 PCA 或拟合得到）
    end: Point
    length: float
    inlier_indices: List[int]


class HoughTransform2D:
    def __init__(self, num_theta: int = 180, rho_resolution: float = 0.1):
        """
        初始化 Hough 变换参数。
        :param num_theta: [0, π) 区间上离散的角度数量。
        :param rho_resolution: ρ 轴的分辨率。
        """
        self.num_theta = num_theta
        self.rho_resolution = rho_resolution
        self.thetas = np.linspace(0, np.pi, num_theta, endpoint=False)

    def _refine_line_with_pca(self, line: Line, inliers: List[Point]) -> None:
        """
        利用 PCA 对 inlier 点重新拟合直线，更新直线端点及长度。
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
        根据两个点构造直线模型，并归一化参数。
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
        对直线的 inlier 点按照在直线方向上的投影值进行聚类，
        如果相邻点之间的投影间隔大于 distance_threshold，则认为是两个独立的 cluster，
        将直线拆分为多个子直线。
        :param line: 检测得到的直线模型。
        :param inlier_data: 每个元素为 (Point, index)。
        :param distance_threshold: 聚类的距离阈值。
        :return: 拆分后的直线列表，如果不需要拆分则返回 [line]。
        """
        # 计算直线方向向量（从 line.start 指向 line.end）
        direction = np.array([line.end.x - line.start.x, line.end.y - line.start.y])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return [line]
        direction = direction / norm

        # 将每个 inlier 投影到直线方向上
        projections = []
        for p, idx in inlier_data:
            vec = np.array([p.x - line.start.x, p.y - line.start.y])
            proj = np.dot(vec, direction)
            projections.append((proj, p, idx))
        # 按投影值排序
        projections.sort(key=lambda x: x[0])

        # 根据投影值间隔进行聚类
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
            # 用 cluster 中首尾两点初步构造直线模型
            new_line = self._compute_line_model(cluster_points[0], cluster_points[-1])
            self._refine_line_with_pca(new_line, cluster_points)
            new_line.inlier_indices = cluster_indices
            new_lines.append(new_line)
        return new_lines

    # def detect(
    #     self,
    #     points: List[Point],
    #     vote_threshold: int = 10,
    #     tolerance: float = 0.1,
    #     max_lines: int = 100,
    #     split_distance_threshold: float = 5.0,
    # ) -> List[Line]:
    #     """
    #     采用迭代 Hough Transform 检测直线：
    #      1. 对剩余点进行投票，并选取最高投票的 bin。
    #      2. 利用该 bin 内的候选点构建初步直线模型。
    #      3. 用该模型对所有剩余点进行 inlier 选取。
    #      4. 利用 _split_line_if_needed 对 inlier 集合进行聚类拆分（类似于 ransac_2d）。
    #      5. 移除 inlier 后继续下一轮检测。

    #     :param points: 2D 点集列表。
    #     :param vote_threshold: 累加器中投票数达到该值才认为直线候选有效。
    #     :param tolerance: 判断点是否属于直线的距离容差。
    #     :param max_lines: maximum number of detected lines.
    #     :param split_distance_threshold: 拆分直线时，inlier 在直线方向上的最大允许间隔。
    #     :return: 检测到的直线列表。
    #     """
    #     detected_lines = []
    #     remaining_points = points.copy()

    #     while (
    #         len(remaining_points) >= vote_threshold and len(detected_lines) < max_lines
    #     ):
    #         # 计算当前剩余点集的 ρ 范围
    #         xs = np.array([p.x for p in remaining_points])
    #         ys = np.array([p.y for p in remaining_points])
    #         diag_len = np.hypot(np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))
    #         rho_min = -diag_len
    #         rho_max = diag_len
    #         rhos = np.arange(rho_min, rho_max, self.rho_resolution)
    #         num_rhos = len(rhos)

    #         # 初始化累加器及记录每个 bin 内的点索引（针对 remaining_points）
    #         accumulator = np.zeros((num_rhos, len(self.thetas)), dtype=int)
    #         votes = [[[] for _ in range(len(self.thetas))] for _ in range(num_rhos)]
    #         cos_t = np.cos(self.thetas)
    #         sin_t = np.sin(self.thetas)

    #         # 对剩余点投票
    #         for idx, p in enumerate(remaining_points):
    #             computed_rhos = p.x * cos_t + p.y * sin_t
    #             bin_indices = np.round(
    #                 (computed_rhos - rho_min) / self.rho_resolution
    #             ).astype(int)
    #             for j, i in enumerate(bin_indices):
    #                 if 0 <= i < num_rhos:
    #                     accumulator[i, j] += 1
    #                     votes[i][j].append(idx)

    #         max_vote = np.max(accumulator)
    #         print(f"max vote: {max_vote}")
    #         if max_vote < vote_threshold:
    #             break

    #         i_max, j_max = np.unravel_index(np.argmax(accumulator), accumulator.shape)
    #         # 根据最高投票的 bin 得到候选直线参数
    #         rho_val = rhos[i_max]
    #         theta_val = self.thetas[j_max]
    #         a = np.cos(theta_val)
    #         b = np.sin(theta_val)
    #         c = -rho_val

    #         # 利用候选直线模型初步选取 inlier（用连续距离判断）
    #         inlier_indices = []
    #         inlier_points = []
    #         for idx, p in enumerate(remaining_points):
    #             d = abs(a * p.x + b * p.y + c)  # 此处 a,b 已为单位向量
    #             if d < tolerance:
    #                 inlier_indices.append(idx)
    #                 inlier_points.append(p)

    #         print(f"number of inliers: {len(inlier_points)}")
    #         # 如果候选 inlier 点不足，则退出
    #         if len(inlier_points) < vote_threshold:
    #             break

    #         # 用候选 inlier 点构造初步直线模型，并精炼（这里先用首尾点构造，再用 PCA 改进）
    #         candidate_line = self._compute_line_model(
    #             inlier_points[0], inlier_points[-1]
    #         )
    #         self._refine_line_with_pca(candidate_line, inlier_points)

    #         # 构造 inlier 数据列表，用于拆分：每个元素 (Point, 在 remaining_points 中的索引)
    #         inlier_data = [(remaining_points[idx], idx) for idx in inlier_indices]
    #         # 调用拆分函数：如果 inlier 分布存在较大间隔，则拆分为多条直线
    #         split_lines = self._split_line_if_needed(
    #             candidate_line, inlier_data, distance_threshold=split_distance_threshold
    #         )
    #         for l in split_lines:
    #             if len(l.inlier_indices) > vote_threshold:
    #                 detected_lines.append(l)

    #         # 移除所有已选出的 inlier 点，继续下一轮迭代
    #         remaining_points = [
    #             p
    #             for idx, p in enumerate(remaining_points)
    #             if idx not in set(inlier_indices)
    #         ]

    #     print(len(detected_lines))
    #     detected_lines = sorted(detected_lines, key=lambda l: len(l.inlier_indices), reverse=True)
    #     return detected_lines

    def detect(
        self,
        points: List[Point],
        vote_threshold: int = 10,
        tolerance: float = 0.1,
        max_lines: int = 100,
        split_distance_threshold: float = 5.0,
    ) -> List[Line]:
        """
        采用迭代 Hough Transform 检测直线：
        1. 对剩余点进行投票，并选取最高投票的 bin。
        2. 直接使用该 bin 内的投票点作为 inliers 构建初步直线模型。
        3. 用 PCA 对候选直线模型进行优化。
        4. 利用 _split_line_if_needed 对 inlier 集合进行聚类拆分（类似于 ransac_2d）。
        5. 移除 inlier 后继续下一轮检测。

        :param points: 2D 点集列表。
        :param vote_threshold: 累加器中投票数达到该值才认为直线候选有效。
        :param tolerance: 保留参数（可与投票数阈值配合调整）。
        :param max_lines: 最大检测直线数。
        :param split_distance_threshold: 拆分直线时，inlier 在直线方向上的最大允许间隔。
        :return: 检测到的直线列表。
        """
        detected_lines = []
        remaining_points = points.copy()

        while (
            len(remaining_points) >= vote_threshold and len(detected_lines) < max_lines
        ):
            # 计算当前剩余点集的 ρ 范围
            xs = np.array([p.x for p in remaining_points])
            ys = np.array([p.y for p in remaining_points])
            diag_len = np.hypot(np.max(xs) - np.min(xs), np.max(ys) - np.min(ys))
            rho_min = -diag_len
            rho_max = diag_len
            rhos = np.arange(rho_min, rho_max, self.rho_resolution)
            num_rhos = len(rhos)

            # 初始化累加器及记录每个 bin 内的点索引（针对 remaining_points）
            accumulator = np.zeros((num_rhos, len(self.thetas)), dtype=int)
            votes = [[[] for _ in range(len(self.thetas))] for _ in range(num_rhos)]
            cos_t = np.cos(self.thetas)
            sin_t = np.sin(self.thetas)

            # 对剩余点投票
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
            # 根据最高投票的 bin 得到候选直线参数
            rho_val = rhos[i_max]
            theta_val = self.thetas[j_max]
            a = np.cos(theta_val)
            b = np.sin(theta_val)
            c = -rho_val

            # 直接使用该 bin 中的投票点作为 inliers
            inlier_indices = votes[i_max][j_max]
            inlier_points = [remaining_points[idx] for idx in inlier_indices]

            # 用 inlier 点构造初步直线模型，并利用 PCA 进行优化
            candidate_line = self._compute_line_model(inlier_points[0], inlier_points[-1])
            self._refine_line_with_pca(candidate_line, inlier_points)

            # 构造 inlier 数据列表，用于拆分：每个元素 (Point, 在 remaining_points 中的索引)
            inlier_data = [(remaining_points[idx], idx) for idx in inlier_indices]
            # 调用拆分函数：如果 inlier 分布存在较大间隔，则拆分为多条直线
            split_lines = self._split_line_if_needed(
                candidate_line, inlier_data, distance_threshold=split_distance_threshold
            )
            for l in split_lines:
                if len(l.inlier_indices) >= vote_threshold:
                    detected_lines.append(l)

            # 移除所有已选出的 inlier 点，继续下一轮迭代
            remaining_points = [
                p for idx, p in enumerate(remaining_points) if idx not in set(inlier_indices)
            ]

        print(f"Total detected lines: {len(detected_lines)}")
        detected_lines = sorted(detected_lines, key=lambda l: len(l.inlier_indices), reverse=True)
        return detected_lines
