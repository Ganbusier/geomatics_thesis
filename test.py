import numpy as np
import time
from plyfile import PlyData
from typing import List
from ransac_2d import Ransac2D, Point
from ransac_3d import Ransac3D, Point3D, Line3D
from ransac_plane import RansacPlane3D, Point3D, Plane3D
from hough_2d import HoughTransform2D
from sklearn.cluster import DBSCAN

import rerun as rr


def project(points: np.array, depth_thres: float = 0.1) -> np.ndarray:

    # centralize points
    points_centered = points - np.mean(points, axis=0)

    # calculate covariance matrix
    cov_matrix = np.cov(points_centered, rowvar=False)

    # solve the covariance matrix for eigen values and vectors
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # choose the first two biggest eigen values (the main 2D plane)
    sorted_indices = np.argsort(eigenvalues)[::-1]
    principal_components = eigenvectors[:, [sorted_indices[0], sorted_indices[1]]]
    depth_component = eigenvectors[:, sorted_indices[2]]

    # calculate depth values
    depth_values = points_centered.dot(depth_component)
    front_mask = depth_values <= depth_thres * np.max(depth_values)

    # project 3D points to 2D plane and keep the depth component
    points_2d = points_centered[front_mask].dot(principal_components)

    return points_2d


def convert_to_points_for_detection(points_2d: np.ndarray) -> List[Point]:
    return [Point(x=float(p[0]), y=float(p[1])) for p in points_2d]


def main(input_model: str, detect_mode: int = 0) -> None:
    # read ply data
    ply_data = PlyData.read(input_model)
    vertices = ply_data["vertex"]
    vx = vertices["x"]
    vy = vertices["y"]
    vz = vertices["z"]

    # offset point clouds
    minx = np.min(vx)
    miny = np.min(vy)
    minz = np.min(vz)
    vx = [xi - minx for xi in vx]
    vy = [yi - miny for yi in vy]
    vz = [zi - minz for zi in vz]

    # construct 2d points for ransace2D
    points = np.array([vx, vy, vz]).T
    points_2d = project(points=points, depth_thres=1.0)
    points_for_detection = convert_to_points_for_detection(points_2d)
    points_for_detection3D = [Point3D(p[0], p[1], p[2]) for p in points]

    # perform 2D ransac
    if detect_mode == 0:
        start_time = time.time()

        # ransac = Ransac2D()
        # ransac_lines = ransac.detect_2(
        #     points=points_for_detection,
        #     max_iterations=200,
        #     min_inliers=5,
        #     tolerance=0.05,
        #     split_distance_threshold=1.0,
        # )

        ransac = RansacPlane3D()
        ransac_planes = ransac.detect_planes(
            points=points_for_detection3D,
            distance_threshold=0.1,
            min_inliers=50,
            confidence=0.99,
        )

        end_time = time.time()
        print(f"Total process time: {end_time - start_time:.4f} seconds")

        # rerun_lines = []
        # for line in ransac_lines:
        #     line_start = (line.start.x, line.start.y)
        #     line_end = (line.end.x, line.end.y)
        #     rerun_lines.append((line_start, line_end))

        rr.init("RANSAC logger", spawn=True)
        for i in range(len(ransac_planes)):
            plane = ransac_planes[i]
            rr_points = [points[idx] for idx in plane.inlier_indices]
            rr.log(f"points{i}", rr.Points3D(rr_points, radii=0.1))
        # rr.log("points2D", rr.Points2D(points_2d, radii=0.1))
        # rr.log("lines2D", rr.LineStrips2D(rerun_lines, radii=0.2))

    # perform 2D hough transform
    elif detect_mode == 1:
        start_time = time.time()

        hough = HoughTransform2D(num_theta=180, rho_resolution=0.1)
        hough_lines = hough.detect(
            points=points_for_detection,
            vote_threshold=4,
            split_distance_threshold=1.0
        )

        end_time = time.time()
        print(f"Total process time: {end_time - start_time:.4f} seconds")

        rerun_lines = []
        for line in hough_lines:
            line_start = (line.start.x, line.start.y)
            line_end = (line.end.x, line.end.y)
            rerun_lines.append((line_start, line_end))

        rr.init("Hough logger", spawn=True)
        rr.log("points2D", rr.Points2D(points_2d, radii=0.1))
        rr.log("lines2D", rr.LineStrips2D(rerun_lines, radii=0.1))

    else:
        print("Wrong detect mode input.")


if __name__ == "__main__":
    input_pylon = "./resources/2024_C_44HZ1_14_pylon.ply"
    input_line = "./resources/2024_C_44HZ1_14_line.ply"

    main(input_pylon, detect_mode=1)
