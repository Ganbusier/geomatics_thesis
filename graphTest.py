import numpy as np
import open3d as o3d
import rerun as rr
from graph import Graph
import argparse
import logging


def offset_point_cloud(points):
    """offset point cloud to origin
    
    args:
        points: numpy array, shape (n, 3), point cloud coordinates
        
    returns:
        tuple: (offset_points, offset), offset points and offset vector
    """
    # compute center
    center = np.min(points, axis=0)
    
    # offset points to origin
    offset_points = points - center
    
    return offset_points, center

def visualize_graph(points, graph, graph_name, edge_labels=None):
    """visualize graph using rerun
    
    args:
        points: numpy array, shape (n, 3), point cloud coordinates
        graph: networkx.Graph, graph to visualize
        graph_name: str, name of the graph for rerun logging
        edge_labels: dict, optional edge labels for coloring
    """
    # log edges
    edges = np.array(list(graph.edges()))
    if len(edges) > 0:
        line_segments = [(points[edge[0]], points[edge[1]]) for edge in edges]
        
        # if edge labels are provided, color edges by cluster
        if edge_labels is not None:
            colors = []
            for edge in edges:
                label = edge_labels[tuple(edge)]
                if label == -1:  # noise points
                    colors.append([1, 0, 0])  # red
                else:
                    # generate a unique color for each cluster
                    hue = label / (max(edge_labels.values()) + 1)
                    colors.append([hue, 1, 1])  # HSV color space
            rr.log(f"{graph_name}", rr.LineStrips3D(line_segments, radii=0.01, colors=colors))
        else:
            rr.log(f"{graph_name}", rr.LineStrips3D(line_segments, radii=0.01))

def visualize_edge_vectors(points, graph, graph_name, edge_labels=None, arrow_length=5.0):
    """visualize edge direction vectors using rerun
    
    args:
        points: numpy array, shape (n, 3), point cloud coordinates
        graph: networkx.Graph, graph to visualize
        graph_name: str, name of the graph for rerun logging
        edge_labels: dict, optional edge labels for coloring
        arrow_length: float, length of the arrow for visualization
    """
    # get edges
    edges = np.array(list(graph.edges()))
    if len(edges) == 0:
        return
    
    # compute edge vectors
    edge_vectors = []
    for edge in edges:
        v1 = points[edge[0]]
        v2 = points[edge[1]]
        # compute vector from v1 to v2
        vector = v2 - v1
        norm = np.linalg.norm(vector)
        if norm > 0:
            # normalize vector
            vector = vector / norm
            # ensure vector points in positive z direction
            if vector[2] < 0:
                vector = -vector
            edge_vectors.append(vector)
    
    edge_vectors = np.array(edge_vectors)
    
    # create arrow endpoints (all starting from origin)
    arrow_starts = np.zeros((len(edges), 3))  # all vectors start from origin
    arrow_ends = edge_vectors * arrow_length
    
    # create line segments for arrows
    line_segments = np.stack([arrow_starts, arrow_ends], axis=1)
    
    # if edge labels are provided, color arrows by cluster
    if edge_labels is not None:
        colors = []
        for edge in edges:
            label = edge_labels[tuple(edge)]
            if label == -1:  # noise points
                colors.append([1, 0, 0]) # red
            else:
                # generate a unique color for each cluster
                hue = label / (max(edge_labels.values()) + 1)
                colors.append([hue, 1, 1])  # HSV color space
        rr.log(f"{graph_name}_vectors", rr.LineStrips3D(line_segments, radii=0.01, colors=colors))
    else:
        rr.log(f"{graph_name}_vectors", rr.LineStrips3D(line_segments, radii=0.01))

def load_direction_vectors(ply_file):
    """load direction vectors from ply file, all vectors start from origin
    
    args:
        ply_file: str, input ply file path
        
    returns:
        list: list of direction vectors
    """
    try:
        # read ply file
        pcd = o3d.io.read_point_cloud(ply_file)
        points = np.asarray(pcd.points)
        
        # compute direction vectors from adjacent points
        direction_vectors = []
        for i in range(0, len(points)-1, 2):  # step by 2 to get pairs
            v1 = points[i]
            v2 = points[i+1]
            vector = v2 - v1
            # normalize vector
            norm = np.linalg.norm(vector)
            if norm > 0:
                vector = vector / norm
                # ensure vector points in positive z direction
                if vector[2] < 0:
                    vector = -vector
                # create line segment from origin
                direction_vectors.append([[0, 0, 0], vector])
        
        logging.info(f"loaded {len(direction_vectors)} direction vectors from {ply_file}")
        return direction_vectors
        
    except Exception as e:
        logging.error(f"failed to load direction vectors: {str(e)}")
        raise

def visualize_line_segments(segments, name, color=None):
    """visualize line segments using rerun
    
    args:
        segments: numpy array, shape (n, 2, 3), line segments
        name: str, name for rerun logging
        color: list, optional RGB color
    """
    if len(segments) > 0:
        if color is None:
            rr.log(name, rr.LineStrips3D(segments, radii=0.01))
        else:
            rr.log(name, rr.LineStrips3D(segments, radii=0.01, colors=[color] * len(segments)))

def main():
    # set up logging
    logging.basicConfig(level=logging.INFO,
                       format='%(asctime)s - %(levelname)s - %(message)s')
    
    # parse command line arguments
    parser = argparse.ArgumentParser(description='visualize different graph structures')
    parser.add_argument('input_file', help='input point cloud file path')
    parser.add_argument('--directions-file', default='resources/directions.ply',
                       help='ply file containing direction vectors (default: resources/directions.ply)')
    parser.add_argument('--k', type=int, default=16,
                       help='number of neighbors for knn graph (default: 16)')
    parser.add_argument('--distance-threshold', type=float, default=2.0,
                       help='maximum edge length threshold for delaunay graph (default: 2.0)')
    parser.add_argument('--coplanar-threshold', type=float, default=1e-6,
                       help='threshold for coplanar points detection (default: 1e-6)')
    parser.add_argument('--angle-threshold', type=float, default=5,
                       help='maximum angle difference in degrees between edges for clustering (default: 5)')
    parser.add_argument('--min-samples', type=int, default=5,
                       help='minimum number of samples in a neighborhood for dbscan (default: 5)')
    parser.add_argument('--arrow-length', type=float, default=0.5,
                       help='length of arrows for vector visualization (default: 0.5)')
    parser.add_argument('--align-angle-threshold', type=float, default=30,
                       help='maximum angle difference in degrees for edge alignment (default: 30)')
    args = parser.parse_args()
    
    try:
        # load point cloud
        pcd = o3d.io.read_point_cloud(args.input_file)
        points = np.asarray(pcd.points)
        
        # offset point cloud to origin
        logging.info("offsetting point cloud to origin...")
        offset_points, center = offset_point_cloud(points)
        logging.info(f"point cloud offset by: {center}")
        
        # initialize rerun
        rr.init("graph_visualization", spawn=True)
        
        # log offset points
        rr.log("offset_points", rr.Points3D(offset_points, radii=0.05))
        
        # create graph object with offset points
        graph = Graph(offset_points)
        
        # build and visualize knn graph
        logging.info("building knn graph...")
        knn_graph = graph.build_knn_graph(k=args.k)
        visualize_graph(offset_points, knn_graph, "knn_graph")
        
        # load direction vectors from ply file
        logging.info("loading direction vectors...")
        main_directions = load_direction_vectors(args.directions_file)
        visualize_line_segments(main_directions, "main_directions", color=[0, 0, 1])  # blue color
        
        # align edges to main directions
        logging.info("aligning edges to main directions...")
        aligned_segments = graph.align_edges_to_directions(
            knn_graph, 
            main_directions, 
            angle_threshold=args.align_angle_threshold
        )
        
        # visualize aligned segments
        visualize_line_segments(aligned_segments, "aligned_segments", color=[0, 1, 0])  # green color
        
        # visualize direction vectors
        visualize_edge_vectors(offset_points, knn_graph, "direction_vectors", arrow_length=args.arrow_length)
        
    except Exception as e:
        logging.error(f"visualization failed: {str(e)}")
        raise

if __name__ == "__main__":
    main()
