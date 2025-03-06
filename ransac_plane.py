import numpy as np
import random
from dataclasses import dataclass
from typing import List, Tuple, Optional


@dataclass
class Point3D:
    x: float
    y: float
    z: float 

@dataclass
class Plane3D:
    coefficients: np.ndarray  # [A, B, C, D] --> Ax + By + Cz + D = 0
    inlier_indices: List[int]
    normal: np.ndarray = None  # normalized normal (A, B, C)
    centroid: Point3D = None   # in-plane center of mass


class RansacPlane3D:
    def __init__(self):
        self.rng = random.Random()
    
    def _compute_plane_model(self, p1: Point3D, p2: Point3D, p3: Point3D) -> Plane3D:
        """Compute plane model use three points"""
        v1 = np.array([p2.x - p1.x, p2.y - p1.y, p2.z - p1.z])
        v2 = np.array([p3.x - p1.x, p3.y - p1.y, p3.z - p1.z])
        normal = np.cross(v1, v2)
        norm = np.linalg.norm(normal)
        
        if norm < 1e-6:  # handle collinearity
            return Plane3D(coefficients=np.zeros(4), inlier_indices=[])
        
        normal /= norm  # normalize plane normal
        D = -np.dot(normal, [p1.x, p1.y, p1.z])
        return Plane3D(
            coefficients=np.append(normal, D),
            inlier_indices=[],
            normal=normal
        )

    def _distance_to_plane(self, p: Point3D, plane: Plane3D) -> float:
        return abs(np.dot(plane.coefficients[:3], [p.x, p.y, p.z]) + plane.coefficients[3])

    def _refine_plane_with_pca(self, plane: Plane3D, points: List[Point3D]):
        if len(points) < 3:
            return
        
        points_array = np.array([[p.x, p.y, p.z] for p in points])
        centroid = np.mean(points_array, axis=0)
        centered = points_array - centroid
        
        # covariance matrix
        cov_matrix = np.cov(centered.T)
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # smallest eigen value represents the plane normal
        normal = eigenvectors[:, np.argmin(eigenvalues)]
        normal /= np.linalg.norm(normal)
        
        # update plane equation parameters
        D = -np.dot(normal, centroid)
        plane.coefficients = np.append(normal, D)
        plane.normal = normal
        plane.centroid = Point3D(*centroid)

    def detect_planes(
        self,
        points: List[Point3D],
        distance_threshold: float = 0.05,
        min_inliers: int = 50,
        confidence: float = 0.99
    ) -> List[Plane3D]:
        remaining_indices = list(range(len(points)))
        planes = []
        
        while len(remaining_indices) >= min_inliers:
            best_plane = None
            best_inliers = []
            
            # dynamic iterations
            max_iterations = min(int(np.floor(len(remaining_indices) / 3)), 200)
            inlier_ratio = len(best_inliers) / len(remaining_indices) if best_inliers else 0.01
            k = int(np.log(1 - confidence) / np.log(1 - inlier_ratio**3)) if inlier_ratio > 0 else max_iterations
            current_iterations = min(k, max_iterations) 
            
            print(f"Remaining indices: {len(remaining_indices)}, current iterations: {current_iterations}")
            
            for _ in range(current_iterations):
                sample_indices = self.rng.sample(remaining_indices, 3)
                p1, p2, p3 = [points[i] for i in sample_indices]
                
                plane = self._compute_plane_model(p1, p2, p3)
                if np.linalg.norm(plane.normal) < 1e-6:
                    continue
                
                inliers = [
                    i for i in remaining_indices
                    if self._distance_to_plane(points[i], plane) <= distance_threshold
                ]
                
                if len(inliers) > len(best_inliers):
                    plane.inlier_indices = inliers
                    best_plane = plane
                    best_inliers = inliers
            
            if best_plane and len(best_inliers) >= min_inliers:
                inlier_points = [points[i] for i in best_inliers]
                self._refine_plane_with_pca(best_plane, inlier_points)
                
                remaining_indices = [i for i in remaining_indices if i not in best_inliers]
                planes.append(best_plane)
            else:
                break
        
        return sorted(planes, key=lambda p: len(p.inlier_indices), reverse=True)
