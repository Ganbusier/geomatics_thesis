import laspy
from plyfile import PlyData, PlyElement
import numpy as np
from tqdm import tqdm

input_file = "/Volumes/T7 Shield/TUD/2024_C_44HZ1.LAZ"
output_file = "./resources/2024_C_44HZ1_14.ply"

filtered_points = []

with laspy.open(input_file) as fh:
    for points in tqdm(fh.chunk_iterator(100000), total= fh.header.point_count // 100000, desc="Processing points"):
        coords = np.vstack((points.x, points.y, points.z)).T
        intensity = points.intensity
        classification = points.classification

        mask = classification == 14
        filtered_points.extend(np.column_stack((coords[mask], intensity[mask], classification[mask])))

filtered_points = np.array(filtered_points)
num_points = len(filtered_points)
vertex = np.array(
    [(x, y, z, intensity, sem_class) for x, y, z, intensity, sem_class in filtered_points], 
    dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("intensity", "f4"), ("sem_class", "i4")]
)
ply = PlyData([PlyElement.describe(vertex, "vertex")], text=True)
ply = PlyData([PlyElement.describe(vertex, "vertex", num_points)], text=True)
ply.write(output_file)

print(f"Filtered point cloud with semantics saved to {output_file}")