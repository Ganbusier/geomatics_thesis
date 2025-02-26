import numpy as np
import time
from plyfile import PlyData
from typing import List
from ransac_2d import Ransac2D, Point
from hough_2d import HoughTransform2D

import matplotlib.pyplot as plt


def project(points: np.array) -> np.ndarray:

    # centralize points
    points_centered = points - np.mean(points, axis=0)

    # calculate covariance matrix
    cov_matrix = np.cov(points_centered, rowvar=False)

    # solve the covariance matrix for eigen values and vectors
    eigenvalues, eigenvectors = np.linalg.eig(cov_matrix)

    # choose the first two biggest eigen values (the main 2D plane)
    sorted_indices = np.argsort(eigenvalues)[::-1]
    principal_components = eigenvectors[:, [sorted_indices[0], sorted_indices[2]]]

    # project 3D points to 2D plane
    points_2d = points_centered.dot(principal_components)

    # # visualize
    # plt.figure(figsize=(8, 6))
    # plt.scatter(points_2d[:, 0], points_2d[:, 1], s=1, c='b')
    # plt.title("2D PCA Projection of the Point Cloud")
    # plt.xlabel("Principal Component 1")
    # plt.ylabel("Principal Component 2")
    # plt.axis("equal")
    # plt.show()

    return points_2d


def convert_to_points_for_detection(points_2d: np.ndarray) -> List[Point]:
    return [Point(x=float(p[0]), y=float(p[1])) for p in points_2d]


def visualize(points_2d: np.ndarray, lines: List) -> None:
    plt.figure(figsize=(10, 8))
    plt.scatter(points_2d[:, 1], points_2d[:, 0], s=2, c="gray", label="Points")

    for idx, line in enumerate(lines):
        xs = [line.start.x, line.end.x]
        ys = [line.start.y, line.end.y]
        plt.plot(ys, xs, lw=2)

        # inlier_points = np.array(
        #     [[points_2d[i, 0], points_2d[i, 1]] for i in line.inlier_indices]
        # )
        # plt.scatter(inlier_points[:, 1], inlier_points[:, 0], s=10, c="red")

    plt.title("2D PCA Projection and Line Detection")
    plt.xlabel("Principal Component 2")
    plt.ylabel("Principal Component 1")
    plt.legend()
    plt.axis("equal")
    plt.show()


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
    points_2d = project(points)
    points_for_detection = convert_to_points_for_detection(points_2d)

    # perform 2D ransac
    if detect_mode == 0:
        start_time = time.time()

        ransac = Ransac2D()
        ransac_lines = ransac.detect_2(
            points=points_for_detection,
            max_iterations=1000,
            min_inliers=500,
            tolerance=0.05,
            split_distance_threshold=1.0,
        )

        end_time = time.time()
        print(f"Total process time: {end_time - start_time:.4f} seconds")
        
        visualize(points_2d, ransac_lines)

    # perform 2D hough transform
    elif detect_mode == 1:
        start_time = time.time()

        hough = HoughTransform2D(num_theta=180, rho_resolution=0.1)
        hough_lines = hough.detect(
            points=points_for_detection,
            vote_threshold=10,
            tolerance=0.05,
            max_lines=1000,
            split_distance_threshold=2.0,
        )

        end_time = time.time()
        print(f"Total process time: {end_time - start_time:.4f} seconds")

        visualize(points_2d, hough_lines)
    else:
        print("Wrong detect mode input.")


if __name__ == "__main__":
    input_pylon = "./resources/2024_C_44HZ1_14_pylon.ply"
    input_line = "./resources/2024_C_44HZ1_14_line.ply"

    main(input_pylon, detect_mode=0)
