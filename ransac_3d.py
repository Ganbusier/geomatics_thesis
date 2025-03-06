import numpy as np
import random
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class Point3D:
    x: float
    y: float
    z: float  # 添加第三维坐标


@dataclass
class Line3D:
    direction: np.ndarray  # 三维方向向量
    point: Point3D  # 直线经过的基准点
    inlier_indices: List[int]
    start: Point3D = None  # 线段起点（根据内点范围确定）
    end: Point3D = None  # 线段终点


class Ransac3D:
    def __init__(self):
        self.rng = random.Random()

    def _compute_line_model(self, p1: Point3D, p2: Point3D) -> Line3D:
        """通过两点确定三维直线模型"""
        direction = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
        norm = np.linalg.norm(direction)
        if norm < 1e-6:
            return Line3D(direction=np.zeros(3), point=p1, inlier_indices=[])
        return Line3D(
            direction=direction / norm,  # 归一化方向向量
            point=p1,  # 使用第一个点作为基准点
            inlier_indices=[],
        )

    def _distance_to_line(self, p: Point3D, line: Line3D) -> float:
        """计算点到三维直线的距离"""
        vec_p = np.array([p.x - line.point.x, p.y - line.point.y, p.z - line.point.z])
        cross = np.cross(vec_p, line.direction)
        return np.linalg.norm(cross)

    def _refine_line_with_pca(self, line: Line3D, inliers: List[Point3D]):
        """使用PCA优化直线方向"""
        if len(inliers) < 2:
            return

        # 转换为numpy数组
        points = np.array([(p.x, p.y, p.z) for p in inliers])
        centroid = np.mean(points, axis=0)

        # 计算协方差矩阵
        centered = points - centroid
        cov = centered.T @ centered / len(points)

        # 特征分解
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        main_direction = eigenvectors[:, np.argmax(eigenvalues)]  # 最大特征值对应方向

        # 更新方向向量
        line.direction = main_direction / np.linalg.norm(main_direction)

        # 计算线段端点
        projections = centered.dot(main_direction)
        min_idx = np.argmin(projections)
        max_idx = np.argmax(projections)
        line.start = Point3D(*points[min_idx])
        line.end = Point3D(*points[max_idx])

    def _split_line_if_needed(
        self,
        line: Line3D,
        inlier_data: List[Tuple[Point3D, int]],
        distance_threshold: float = 5.0,
    ) -> List[Line3D]:
        """
        三维线段分割函数，基于投影距离阈值将内点聚类为多个线段
        inlier_data: 元组列表，每个元素为(Point3D, 原始索引)
        """
        # 计算三维方向向量
        start_coord = np.array([line.start.x, line.start.y, line.start.z])
        end_coord = np.array([line.end.x, line.end.y, line.end.z])
        direction = end_coord - start_coord
        norm = np.linalg.norm(direction)

        # 处理退化情况（零向量或接近零向量）
        if norm < 1e-6:
            return [line]
        direction = direction / norm

        # 投影计算（三维向量点积）
        projections = []
        for p, idx in inlier_data:
            point_coord = np.array([p.x, p.y, p.z])
            vec = point_coord - start_coord
            proj = np.dot(vec, direction)  # 标量投影值
            projections.append((proj, p, idx))

        # 按投影值排序（沿直线方向）
        projections.sort(key=lambda x: x[0])

        # 基于投影距离的聚类分割
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

        # 三维PCA优化分割后的线段
        new_lines = []
        for cluster in clusters:
            if len(cluster) < 2:
                continue

            # 提取三维点集和原始索引
            cluster_points = [p for (_, p, _) in cluster]
            cluster_indices = [idx for (_, _, idx) in cluster]

            # 构造初始线段（使用首末点）
            new_line = self._compute_line_model(cluster_points[0], cluster_points[-1])

            # PCA优化方向向量（使用网页3的RANSAC平面分割原理）
            points_array = np.array([[p.x, p.y, p.z] for p in cluster_points])
            centroid = np.mean(points_array, axis=0)
            cov_matrix = np.cov(points_array.T)
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
            main_direction = eigenvectors[:, np.argmax(eigenvalues)]

            # 更新线段参数（参考网页8的PCA优化方法）
            new_line.direction = main_direction / np.linalg.norm(main_direction)
            projections = (points_array - centroid) @ main_direction
            min_idx = np.argmin(projections)
            max_idx = np.argmax(projections)
            new_line.start = Point3D(*points_array[min_idx])
            new_line.end = Point3D(*points_array[max_idx])
            new_line.inlier_indices = cluster_indices

            new_lines.append(new_line)

        return new_lines

    def detect(
        self,
        points: List[Point3D],
        max_iterations: int = 1000,
        min_inliers: int = 20,
        tolerance: float = 0.1,
        split_distance_threshold: float = 2.0,
    ) -> List[Line3D]:
        remaining_indices = list(range(len(points)))
        lines = []

        while min_inliers >= 5:
            print(
                f"Remaining indices: {len(remaining_indices)}, current min inliers: {min_inliers}"
            )
            candidate_lines = []

            iter = 0
            while iter < max_iterations and len(remaining_indices) >= min_inliers:
                # detect valid lines
                idx1, idx2 = self.rng.sample(remaining_indices, k=2)
                p1, p2 = points[idx1], points[idx2]

                dx, dy, dz = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
                distance = np.sqrt(dx**2 + dy**2 + dz**2)
                if distance < 1e-6 and distance > 1.0:
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
                if len(candidate_inliers) < min_inliers:
                    iter += 1
                    continue

                inlier_points = [points[idx] for idx in candidate_inliers]
                self._refine_line_with_pca(candidate_line, inlier_points)
                inlier_data = [(points[idx], idx) for idx in candidate_inliers]
                split_lines = self._split_line_if_needed(
                    candidate_line,
                    inlier_data,
                    distance_threshold=split_distance_threshold,
                )
                valid_split_line_indices = []
                for l in split_lines:
                    if len(l.inlier_indices) > min_inliers:
                        candidate_lines.append(l)
                        valid_split_line_indices.extend(l.inlier_indices)

                remaining_indices = [
                    idx
                    for idx in remaining_indices
                    if idx not in set(valid_split_line_indices)
                ]

            lines.extend(candidate_lines)
            min_inliers = round(min_inliers * 0.9)

        return sorted(lines, key=lambda l: len(l.inlier_indices), reverse=True)

    def detect_2(
        self,
        points: List[Point3D],
        probability: float = 0.99,
        min_points: int = 2,
        min_inliers: int = 5,
        tolerance: float = 0.1,
        split_distance_thres: float = 3.0,
    ):
        remaining_indices = list(range(len(points)))
        lines = []

        t = 0.1
        k = round(abs(np.log(1 - probability) / np.log(1 - np.power(t, min_points))))

        while len(remaining_indices) >= min_inliers:
            print(f"Remaining indices: {len(remaining_indices)}")
            best = []
            best_inliers = []

            for _ in range(1000):
                idx1, idx2 = self.rng.sample(remaining_indices, k=2)
                p1, p2 = points[idx1], points[idx2]

                dx, dy, dz = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
                distance = np.sqrt(dx**2 + dy**2 + dz**2)
                if distance < 1e-6 and distance > 1.0:
                    continue

                candidate_line = self._compute_line_model(p1, p2)
                candidate_line.inlier_indices = [
                    idx
                    for idx in remaining_indices
                    if self._distance_to_line(points[idx], candidate_line) < tolerance
                ]
                if len(candidate_line.inlier_indices) < min_inliers:
                    continue

                candidate_inlier_points = [points[idx] for idx in candidate_line.inlier_indices]
                self._refine_line_with_pca(candidate_line, candidate_inlier_points)

                inlier_data = [(points[idx], idx) for idx in candidate_line.inlier_indices]
                split_lines = self._split_line_if_needed(
                    candidate_line, inlier_data, split_distance_thres
                )
                valid_lines = []
                valid_inliers = []
                for l in split_lines:
                    if len(l.inlier_indices) > min_inliers:
                        valid_lines.append(l)
                        valid_inliers.extend(l.inlier_indices)

                if len(valid_inliers) > len(best_inliers):
                    best = valid_lines
                    best_inliers = valid_inliers
                    t = len(valid_inliers) / len(remaining_indices)
                    k = round(abs(np.log(1 - probability) / np.log(1 - np.power(t, min_points))))

            if best and len(best_inliers) >= min_inliers:
                for l in best:
                    inlier_points = [points[idx] for idx in l.inlier_indices]
                    self._refine_line_with_pca(l, inlier_points)
                    lines.append(l)

                remaining_indices = [
                    idx for idx in remaining_indices if idx not in set(best_inliers)
                ]
            else:
                break

        print(len(lines))
        return lines
