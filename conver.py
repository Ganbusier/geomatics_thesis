import numpy as np
import open3d as o3d
import logging

def obj_to_ply(obj_file, ply_file):
    """convert obj file containing bspline curves to ply file with curve endpoints
    
    args:
        obj_file: str, input obj file path
        ply_file: str, output ply file path
    """
    try:
        # read obj file
        vertices = []
        curve_endpoints = []
        
        with open(obj_file, 'r') as f:
            for line in f:
                if line.startswith('v '):  # vertex
                    # parse vertex coordinates
                    coords = line[2:].strip().split()
                    vertex = [float(x) for x in coords]
                    vertices.append(vertex)
                elif line.startswith('curv '):  # curve definition
                    # parse curve indices
                    parts = line[5:].strip().split()
                    if len(parts) >= 4:  # ensure we have enough parts
                        indices = [int(x) - 1 for x in parts[2:]]  # obj indices start from 1
                        # only keep start and end points
                        curve_endpoints.extend([vertices[indices[0]], vertices[indices[-1]]])
        
        # save as ASCII ply file
        with open(ply_file, 'w') as f:
            # write header
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {len(curve_endpoints)}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("end_header\n")
            
            # write points
            for point in curve_endpoints:
                f.write(f"{point[0]} {point[1]} {point[2]}\n")
        
        logging.info(f"converted {obj_file} to {ply_file}")
        logging.info(f"  number of vertices: {len(vertices)}")
        logging.info(f"  number of curve endpoints: {len(curve_endpoints)}")
        
    except Exception as e:
        logging.error(f"failed to convert obj to ply: {str(e)}")
        raise

if __name__ == "__main__":
    # set up logging
    logging.basicConfig(level=logging.INFO,
                       format='%(asctime)s - %(levelname)s - %(message)s')
    
    # example usage
    obj_file = "./resources/directions.obj"
    ply_file = "./resources/directions.ply"
    obj_to_ply(obj_file, ply_file)
