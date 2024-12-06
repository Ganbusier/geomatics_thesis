import laspy
from plyfile import PlyData, PlyElement
import numpy as np
from tqdm import tqdm


def laz_process(input_file: str, output_file: str):
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


def ply_to_xyz(input_file: str, output_file: str, offset: bool = False):
    ply_data = PlyData.read(input_file)
    vertices = ply_data["vertex"]
    x = vertices["x"]
    y = vertices["y"]
    z = vertices["z"]

    if offset:
        min_x = np.min(x)
        min_y = np.min(y)
        min_z = np.min(z)
        x = [xi - min_x for xi in x]
        y = [yi - min_y for yi in y]
        z = [zi - min_z for zi in z]
        print(f"Offset: ({min_x}, {min_y}, {min_z})")

    with open(output_file, "w") as f:
        for i in range(len(x)):
            f.write(f"{x[i]} {y[i]} {z[i]}\n")

    print(f"XYZ point cloud saved to {output_file}")


def xyz_to_ply(input_file: str, output_file: str):
    pwn_vector = []
    with open(input_file, "r") as f:
        for line in f:
            data = line.strip().split()
            if len(data) == 6:
                x, y, z, nx, ny, nz = map(float, data)
                pwn_vector.append((x, y, z, nx, ny, nz))
    vertex_array = np.array(
        pwn_vector, 
        dtype=[
            ("x", "f4"), ("y", "f4"), ("z", "f4"), 
            ("nx", "f4"), ("ny", "f4"), ("nz", "f4")
        ]
    )
    ply_element = PlyElement.describe(vertex_array, "vertex")
    PlyData([ply_element], text=True).write(output_file)

    print(f"PLY point cloud saved to {output_file}")


if __name__ == "__main__":

    input_file = "./resources/2024_C_44HZ1_14_line_End.xyz"
    output_file = "./resources/2024_C_44HZ1_14_line_End.ply"

    # laz_process(input_file=input_file, output_file=output_file)
    # ply_to_xyz(input_file=input_file, output_file=output_file, offset=True)
    xyz_to_ply(input_file=input_file, output_file=output_file)