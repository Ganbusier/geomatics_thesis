import laspy
from plyfile import PlyData, PlyElement
import numpy as np
from tqdm import tqdm


def laz_process(input_file, output_file):
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

def ply_to_xyz(input_file, output_file):
    ply_data = PlyData.read(input_file)
    vertices = ply_data["vertex"]
    x = vertices["x"]
    y = vertices["y"]
    z = vertices["z"]
    with open(output_file, "w") as f:
        for i in range(len(x)):
            f.write(f"{x[i]} {y[i]} {z[i]}\n")

    print(f"XYZ point cloud saved to {output_file}")


if __name__ == "__main__":

    input_file = "./resources/2024_C_44HZ1_14_pylon.ply"
    output_file = "./resources/2024_C_44HZ1_14_pylon.xyz"

    # laz_process(input_file=input_file, output_file=output_file)
    ply_to_xyz(input_file=input_file, output_file=output_file)