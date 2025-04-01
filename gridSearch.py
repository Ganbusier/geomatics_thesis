import numpy as np
import rerun as rr
from plyfile import PlyData
import open3d as o3d
from scipy.spatial import cKDTree
from collections import deque
import math
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm
from typing import List

class GridSearch:
    def __init__(self, point_cloud: np.ndarray) -> None:
        self.points: np.ndarray = point_cloud
        self.grid_size: float = None
        self.grid: np.ndarray = None
        self.visited: np.ndarray = None
        self.grid_dims: np.ndarray = None
        self.min_coords: np.ndarray = None
        self.voxel_centers: dict = {}

    def compute_grid_size(self, points: np.ndarray) -> None:
        """compute the average distance between points"""
        tree = cKDTree(points)
        distances, _ = tree.query(points, k=2)  # get the distance to the nearest neighbor
        grid_size = np.mean(distances[:, 1])  # average distance
        print(f"Avg distance: {grid_size}")
        self.grid_size = grid_size * 2

    def create_voxel_grid(self, points: np.ndarray) -> None:
        """create the voxel grid and binarize it, and record the center point of each non-empty voxel in the world coordinate"""
        print("create the voxel grid...")
        self.min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        self.grid_dims = np.ceil((max_coords - self.min_coords) / self.grid_size).astype(int)
        
        self.grid = np.zeros(self.grid_dims, dtype=np.int8)
        self.visited = np.zeros(self.grid_dims, dtype=np.bool_)
        self.voxel_centers = {}  # clear the previous record
        
        print("map the points to the grid...")
        for point in tqdm(points, desc="voxelization"):
            idx = np.floor((point - self.min_coords) / self.grid_size).astype(int)
            if np.all(idx >= 0) and np.all(idx < self.grid_dims):
                self.grid[idx[0], idx[1], idx[2]] += 1
                # use the tuple as the key to record the voxel center point
                idx_tuple = (idx[0], idx[1], idx[2])
                if idx_tuple not in self.voxel_centers:
                    # compute the world coordinate of the voxel center
                    center = (idx + 0.5) * self.grid_size + self.min_coords
                    self.voxel_centers[idx_tuple] = center
        
        self.grid = (self.grid > 1).astype(np.int8)        

    def reconstruct(self) -> None:
        """reconstruct line segments from input point cloud"""
        self.compute_grid_size(self.points)
        self.create_voxel_grid(self.points)

        rr.init("gridSearch", spawn=True)
        rr.log("original_points", rr.Points3D(self.points, radii=0.05))

        # log the voxel grid
        if self.voxel_centers:
            voxel_center_points = np.array(list(self.voxel_centers.values()))
            rr.log("voxel_grid/voxel_centers", rr.Points3D(voxel_center_points, radii=0.05))
                
            all_box_lines = []
            for i in range(len(voxel_center_points)):
                center = voxel_center_points[i]
                half_size = self.grid_size / 2
                
                # create the eight vertices of the cube
                corners = np.array([
                    [center[0] - half_size, center[1] - half_size, center[2] - half_size],
                    [center[0] + half_size, center[1] - half_size, center[2] - half_size],
                    [center[0] + half_size, center[1] + half_size, center[2] - half_size],
                    [center[0] - half_size, center[1] + half_size, center[2] - half_size],
                    [center[0] - half_size, center[1] - half_size, center[2] + half_size],
                    [center[0] + half_size, center[1] - half_size, center[2] + half_size],
                    [center[0] + half_size, center[1] + half_size, center[2] + half_size],
                    [center[0] - half_size, center[1] + half_size, center[2] + half_size]
                ])
                
                # define the edges of the cube
                # bottom edge
                all_box_lines.append(np.array([corners[0], corners[1]]))
                all_box_lines.append(np.array([corners[1], corners[2]]))
                all_box_lines.append(np.array([corners[2], corners[3]]))
                all_box_lines.append(np.array([corners[3], corners[0]]))
                
                # top edge
                all_box_lines.append(np.array([corners[4], corners[5]]))
                all_box_lines.append(np.array([corners[5], corners[6]]))
                all_box_lines.append(np.array([corners[6], corners[7]]))
                all_box_lines.append(np.array([corners[7], corners[4]]))
                
                # side edge
                all_box_lines.append(np.array([corners[0], corners[4]]))
                all_box_lines.append(np.array([corners[1], corners[5]]))
                all_box_lines.append(np.array([corners[2], corners[6]]))
                all_box_lines.append(np.array([corners[3], corners[7]]))
                
            # record all the lines as a whole to rerun
            rr.log("voxel_grid/voxel_lines", rr.LineStrips3D(all_box_lines, radii=0.02))

def main():
    # read the point cloud data
    input_pylon = "./resources/2024_C_44HZ1_14_pylon.ply"
    input_pylon_2 = "./resources/pylon_test.ply"
    plydata = PlyData.read(input_pylon_2)
    points = np.vstack([plydata['vertex']['x'],
                       plydata['vertex']['y'],
                       plydata['vertex']['z']]).T
    
    # compute the center of the point cloud
    center = np.mean(points, axis=0)
    
    # move the point cloud to the origin
    points = points - center
    
    # create the reconstructor and execute the reconstruction
    reconstructor = GridSearch(points)
    reconstructor.reconstruct()

if __name__ == "__main__":
    main()
