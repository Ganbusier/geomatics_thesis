import numpy as np
import open3d as o3d
from scipy.spatial import Delaunay
from sklearn.neighbors import NearestNeighbors
from sklearn.cluster import DBSCAN
import networkx as nx
import logging

class Graph:
    def __init__(self, points):
        """initialize graph class
        
        args:
            points: numpy array, shape (n, 3), point cloud coordinates
        """
        self.points = points
        self.n_points = len(points)
        self.knn_graph = None
        self.delaunay_graph = None
        self.combined_graph = None
        
    def build_knn_graph(self, k=16):
        """build knn graph
        
        args:
            k: int, number of nearest neighbors for each point
            
        returns:
            networkx.Graph: knn graph
        """
        try:
            # compute knn
            nbrs = NearestNeighbors(n_neighbors=k, algorithm='kd_tree').fit(self.points)
            distances, indices = nbrs.kneighbors(self.points)
            
            # create graph
            G = nx.Graph()
            G.add_nodes_from(range(self.n_points))
            
            # add edges
            for i in range(self.n_points):
                # add edges to all k nearest neighbors
                for j, dist in zip(indices[i], distances[i]):
                    if i != j:  # avoid self-loops
                        G.add_edge(i, j, weight=dist)
            
            self.knn_graph = G
            logging.info(f"knn graph built successfully with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges")
            return G
            
        except Exception as e:
            logging.error(f"failed to build knn graph: {str(e)}")
            raise
    
    def build_delaunay_graph(self, distance_threshold=2.0):
        """build delaunay triangulation graph
        
        args:
            distance_threshold: float, maximum edge length threshold
            
        returns:
            networkx.Graph: delaunay triangulation graph
        """
        try:
            # build delaunay triangulation
            tri = Delaunay(self.points)
            
            # create graph
            G = nx.Graph()
            G.add_nodes_from(range(self.n_points))
            
            # add edges
            for simplex in tri.simplices:
                # add tetrahedron edges
                for i in range(4):
                    for j in range(i+1, 4):
                        # get edge vertices
                        v1 = self.points[simplex[i]]
                        v2 = self.points[simplex[j]]
                        
                        # compute edge length
                        edge_length = np.linalg.norm(v2 - v1)
                        
                        # add edge if length is within threshold
                        if edge_length <= distance_threshold:
                            G.add_edge(simplex[i], simplex[j], weight=edge_length)
            
            self.delaunay_graph = G
            logging.info(f"delaunay graph built successfully with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges")
            return G
            
        except Exception as e:
            logging.error(f"failed to build delaunay graph: {str(e)}")
            raise
    
    def build_combined_graph(self, k=10, coplanar_threshold=1e-6, distance_threshold=2.0):
        """build combined graph (union of knn and delaunay graphs)
        
        args:
            k: int, number of neighbors for knn graph
            coplanar_threshold: float, threshold for coplanar points detection
            distance_threshold: float, maximum edge length threshold for delaunay graph
            
        returns:
            networkx.Graph: combined graph
        """
        try:
            # build knn and delaunay graphs if not already built
            if self.knn_graph is None:
                self.build_knn_graph(k)
            if self.delaunay_graph is None:
                self.build_delaunay_graph(coplanar_threshold, distance_threshold)
            
            # create combined graph (union of both graphs)
            G = nx.Graph()
            G.add_nodes_from(range(self.n_points))
            
            # add knn graph edges
            for edge in self.knn_graph.edges(data=True):
                G.add_edge(edge[0], edge[1], weight=edge[2]['weight'], type='knn')
            
            # add delaunay graph edges
            for edge in self.delaunay_graph.edges(data=True):
                if not G.has_edge(edge[0], edge[1]):  # if edge doesn't exist, add it
                    G.add_edge(edge[0], edge[1], weight=edge[2]['weight'], type='delaunay')
                else:  # if edge exists, keep the one with smaller weight
                    if edge[2]['weight'] < G[edge[0]][edge[1]]['weight']:
                        G[edge[0]][edge[1]]['weight'] = edge[2]['weight']
                        G[edge[0]][edge[1]]['type'] = 'delaunay'
            
            self.combined_graph = G
            logging.info(f"combined graph built successfully with {G.number_of_nodes()} nodes and {G.number_of_edges()} edges")
            return G
            
        except Exception as e:
            logging.error(f"failed to build combined graph: {str(e)}")
            raise
    
    def save_graph(self, graph, output_file):
        """save graph to file
        
        args:
            graph: networkx.Graph, graph to save
            output_file: str, output file path
        """
        try:
            # convert graph to adjacency matrix
            adj_matrix = nx.adjacency_matrix(graph).todense()
            
            # save as numpy array
            np.save(output_file, adj_matrix)
            logging.info(f"graph saved to: {output_file}")
            
        except Exception as e:
            logging.error(f"failed to save graph: {str(e)}")
            raise
    
    def load_graph(self, input_file):
        """load graph from file
        
        args:
            input_file: str, input file path
            
        returns:
            networkx.Graph: loaded graph
        """
        try:
            # load adjacency matrix
            adj_matrix = np.load(input_file)
            
            # convert to networkx graph
            G = nx.from_numpy_array(adj_matrix)
            logging.info(f"graph loaded from: {input_file}")
            
            return G
            
        except Exception as e:
            logging.error(f"failed to load graph: {str(e)}")
            raise

    def cluster_edges(self, graph, angle_threshold=5, min_samples=5):
        """cluster edges using dbscan based on angle difference
        
        args:
            graph: networkx.Graph, input graph
            angle_threshold: float, maximum angle difference in degrees between edges
            min_samples: int, minimum number of samples in a neighborhood for dbscan
            
        returns:
            dict: edge labels for each edge
        """
        try:
            # get edges
            edges = np.array(list(graph.edges()))
            if len(edges) == 0:
                logging.warning("no edges found in graph")
                return {}
            
            # get edge vectors
            edge_vectors = []
            for edge in edges:
                v1 = self.points[edge[0]]
                v2 = self.points[edge[1]]
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
            
            # convert angle threshold from degrees to radians
            angle_threshold_rad = np.radians(angle_threshold)
            
            # create custom metric function for angle-based distance
            def angle_metric(v1, v2):
                # compute cosine of angle between vectors
                cos_angle = np.abs(np.dot(v1, v2))
                # convert to angle in radians
                angle = np.arccos(np.clip(cos_angle, -1.0, 1.0))
                return angle
            
            # cluster using dbscan with custom metric
            dbscan = DBSCAN(eps=angle_threshold_rad, 
                          min_samples=min_samples,
                          metric=angle_metric)
            labels = dbscan.fit_predict(edge_vectors)
            
            # create edge labels dictionary
            edge_labels = {tuple(edge): label for edge, label in zip(edges, labels)}
            
            # log clustering results
            n_clusters = len(set(labels)) - (1 if -1 in labels else 0)
            n_noise = list(labels).count(-1)
            logging.info(f"edge clustering completed:")
            logging.info(f"  number of clusters: {n_clusters}")
            logging.info(f"  number of noise points: {n_noise}")

            # print the number of edges in each cluster
            for label in set(labels):
                if label != -1:
                    logging.info(f"  number of edges in cluster {label}: {list(labels).count(label)}")
            
            return edge_labels
            
        except Exception as e:
            logging.error(f"failed to cluster edges: {str(e)}")
            raise

    def align_edges_to_directions(self, graph, directions, angle_threshold=30):
        """align edges to the closest direction vector, return aligned line segments
        
        args:
            graph: networkx.Graph, input graph
            directions: list of line segments, each segment is [[0,0,0], direction_vector]
            angle_threshold: float, maximum angle difference in degrees
            
        returns:
            numpy array: array of aligned line segments, shape (n, 2, 3)
        """
        try:
            # extract direction vectors from segments
            direction_vectors = [d[1] for d in directions]
            
            # convert angle threshold to radians
            angle_threshold_rad = np.radians(angle_threshold)
            
            # store aligned line segments
            aligned_segments = []
            
            # process each edge
            for edge in graph.edges():
                # get edge vertices
                v1 = self.points[edge[0]]
                v2 = self.points[edge[1]]
                
                # compute edge vector and move to origin
                edge_vector = v2 - v1
                edge_length = np.linalg.norm(edge_vector)
                
                if edge_length > 0:
                    # normalize edge vector
                    edge_vector = edge_vector / edge_length

                    # find closest direction
                    max_cos_angle = -1
                    closest_direction = None
                    
                    for direction in direction_vectors:
                        # compute cosine of angle between vectors at origin
                        cos_angle = abs(np.dot(edge_vector, direction))
                        if cos_angle > max_cos_angle:
                            max_cos_angle = cos_angle
                            closest_direction = direction
                    
                    # check if angle is within threshold
                    angle = np.arccos(np.clip(max_cos_angle, -1.0, 1.0))
                    if angle <= angle_threshold_rad:
                        # compute rotation matrix
                        rotation_axis = np.cross(edge_vector, closest_direction)
                        if np.linalg.norm(rotation_axis) > 0:
                            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
                            
                            # Rodrigues rotation formula
                            K = np.array([
                                [0, -rotation_axis[2], rotation_axis[1]],
                                [rotation_axis[2], 0, -rotation_axis[0]],
                                [-rotation_axis[1], rotation_axis[0], 0]
                            ])
                            R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * np.dot(K, K)
                            
                            # rotate edge vector at origin
                            rotated_vector = np.dot(R, edge_vector)
                            
                            # compute new endpoints
                            # first compute the rotated vector at origin
                            rotated_vector_at_origin = rotated_vector * edge_length
                            # then translate to original start point
                            mid = (v1 + v2) / 2
                            new_v1 = mid - rotated_vector_at_origin / 2
                            new_v2 = mid + rotated_vector_at_origin / 2
                            
                            # add aligned segment
                            aligned_segments.append([new_v1, new_v2])
            
            # convert to numpy array
            aligned_segments = np.array(aligned_segments)
            
            logging.info(f"aligned edges to directions with {angle_threshold} degree threshold")
            logging.info(f"  created {len(aligned_segments)} aligned segments")
            return aligned_segments
            
        except Exception as e:
            logging.error(f"failed to align edges to directions: {str(e)}")
            raise
