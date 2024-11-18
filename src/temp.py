import laspy
from plyfile import PlyData, PlyElement
import numpy as np
from tqdm import tqdm

input_file = "/Volumes/T7 Shield/TUD/2024_C_44HZ1.LAZ"
output_file = "/Volumes/T7 Shield/TUD/filtered_semantics_14.ply"

filtered_points = []

with laspy.open(input_file) as fh:
    for points in tqdm(fh.chunk_iterator(100000), total= fh.header.point_count // 100000, desc="Processing points"):
        coords = np.vstack((points.x, points.y, points.z)).T
        classification = points.classification

        mask = classification == 14
        filtered_points.extend(np.column_stack((coords[mask], classification[mask])))

filtered_points = np.array(filtered_points)
vertex = np.array(
    [(x, y, z, sem_class) for x, y, z, sem_class in filtered_points], 
    dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("sem_class", "u1")]
)
ply = PlyData([PlyElement.describe(vertex, "vertex")], text=True)
ply.write(output_file)

print(f"Filtered point cloud with semantics saved to {output_file}")