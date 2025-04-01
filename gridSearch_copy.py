import numpy as np
import rerun as rr
from plyfile import PlyData
import open3d as o3d
from scipy.spatial import cKDTree
from collections import deque
import math
from sklearn.neighbors import NearestNeighbors
from tqdm import tqdm

class GridSearch:
    def __init__(self, point_cloud):
        self.points = point_cloud
        self.grid_size = None
        self.grid = None
        self.grid_dims = None
        self.visited = None
        self.segments = []
        self.nodes = []
        self.min_coords = None
        self.augmented_points = None
        self.voxel_centers = {}  # 新增：存储体素索引到世界坐标的映射
        
    def augment_points_with_knn(self, k=16, num_interpolations=5, max_edge_length=2.0):
        """使用KNN图生成额外的点
        
        args:
            k: int, KNN图中的邻居数量
            num_interpolations: int, 每条边上的插值点数量
            max_edge_length: float, 最大边长度，超过此长度的边不进行插值
        """
        print("构建KNN图...")
        nbrs = NearestNeighbors(n_neighbors=k, algorithm='kd_tree').fit(self.points)
        distances, indices = nbrs.kneighbors(self.points)
        
        new_points = []
        print("生成插值点...")
        for i in tqdm(range(len(self.points)), desc="处理点"):
            for j in range(1, k):
                neighbor_idx = indices[i, j]
                if neighbor_idx > i:
                    p1 = self.points[i]
                    p2 = self.points[neighbor_idx]
                    edge_length = np.linalg.norm(p2 - p1)
                    if edge_length > max_edge_length:
                        continue
                    for t in np.linspace(0, 1, num_interpolations + 2)[1:-1]:
                        interpolated_point = p1 + t * (p2 - p1)
                        new_points.append(interpolated_point)
        
        self.augmented_points = np.vstack((self.points, np.array(new_points)))
        print(f"原始点数量: {len(self.points)}")
        print(f"新增点数量: {len(new_points)}")
        print(f"总点数量: {len(self.augmented_points)}")
        print(f"每条边插值点数量: {num_interpolations}")
        print(f"最大边长度阈值: {max_edge_length}")
        
    def compute_grid_size(self, points):
        """计算平均点间距并设置网格大小"""
        tree = cKDTree(points)
        distances, _ = tree.query(points, k=2)  # 获取每个点到最近邻的距离
        avg_distance = np.mean(distances[:, 1])  # 平均点间距
        print(f"Avg distance: {avg_distance}")
        self.grid_size = avg_distance
        return self.grid_size
    
    def create_voxel_grid(self, points):
        """创建体素网格并进行二值化，并记录每个非空体素的中心点世界坐标"""
        print("创建体素网格...")
        self.min_coords = np.min(points, axis=0)
        max_coords = np.max(points, axis=0)
        self.grid_dims = np.ceil((max_coords - self.min_coords) / self.grid_size).astype(int)
        
        self.grid = np.zeros(self.grid_dims, dtype=np.int8)
        self.visited = np.zeros(self.grid_dims, dtype=np.bool_)
        self.voxel_centers = {}  # 清空之前的记录
        
        print("映射点到网格...")
        for point in tqdm(points, desc="体素化"):
            idx = np.floor((point - self.min_coords) / self.grid_size).astype(int)
            if np.all(idx >= 0) and np.all(idx < self.grid_dims):
                self.grid[idx[0], idx[1], idx[2]] += 1
                # 使用元组作为字典键，记录体素中心点
                idx_tuple = (idx[0], idx[1], idx[2])
                if idx_tuple not in self.voxel_centers:
                    # 计算体素中心点的世界坐标
                    center = (idx + 0.5) * self.grid_size + self.min_coords
                    self.voxel_centers[idx_tuple] = center
        
        self.grid = (self.grid > 1).astype(np.int8)
        
    def grid_to_world(self, grid_point):
        """将网格坐标转换为世界坐标"""
        return grid_point * self.grid_size + self.min_coords
    
    def get_neighbors(self, voxel_idx):
        """获取26邻域体素
        
        Returns:
            list: 26个邻居体素的索引列表
        """
        # 预定义26个方向的偏移量
        offsets = np.array([
            [-1,-1,-1], [-1,-1,0], [-1,-1,1],
            [-1,0,-1],  [-1,0,0],  [-1,0,1],
            [-1,1,-1],  [-1,1,0],  [-1,1,1],
            [0,-1,-1],  [0,-1,0],  [0,-1,1],
            [0,0,-1],              [0,0,1],
            [0,1,-1],   [0,1,0],   [0,1,1],
            [1,-1,-1],  [1,-1,0],  [1,-1,1],
            [1,0,-1],   [1,0,0],   [1,0,1],
            [1,1,-1],   [1,1,0],   [1,1,1]
        ])
        
        # 计算所有邻居的索引
        neighbors = voxel_idx + offsets
        
        # 过滤出有效的索引
        valid_mask = np.all((neighbors >= 0) & (neighbors < self.grid_dims), axis=1)
        return neighbors[valid_mask]
    
    def is_seed_point(self, voxel_idx):
        """检测种子点 - 有且仅有相对方向上的两个值为1的点"""
        if self.grid[voxel_idx[0], voxel_idx[1], voxel_idx[2]] != 1:
            return False
            
        neighbors = self.get_neighbors(voxel_idx)
        active_neighbors = [n for n in neighbors if self.grid[n[0], n[1], n[2]] == 1]
        
        # 如果邻居点不是恰好2个，则不是种子点
        if len(active_neighbors) != 2:
            return False
        
        # 计算两个邻居点相对于中心点的偏移量
        offset1 = active_neighbors[0] - voxel_idx
        offset2 = active_neighbors[1] - voxel_idx
        
        # 如果两个偏移量互为相反数，则这两个邻居点在相对方向上
        if (offset1[0] == -offset2[0] and 
            offset1[1] == -offset2[1] and 
            offset1[2] == -offset2[2]):
            return True
        
        return False
    
    def trace_line(self, seed_point, line_id=None):
        """线特征追踪
        
        Args:
            seed_point: 种子点的网格坐标
            line_id: 线段ID，如果为None则自动分配新ID
            
        Returns:
            line_points: 线段点序列（世界坐标）
        """
        # 如果没有指定line_id，则分配一个新的
        if line_id is None:
            line_id = len(self.segments) + 1
            
        current_idx = seed_point.copy()  # 当前网格索引
        idx_tuple = tuple(current_idx)
        
        # 使用预计算的体素中心点世界坐标，如果没有则计算
        if idx_tuple in self.voxel_centers:
            center_point = self.voxel_centers[idx_tuple]
        else:
            center_point = self.grid_to_world(current_idx)
            
        line_points = [center_point]  # 使用体素中心点作为线段点
        
        # 初始化搜索方向
        neighbors = self.get_neighbors(seed_point)
        active_neighbors = [n for n in neighbors if self.grid[n[0], n[1], n[2]] == 1 
                          and not self.visited[n[0], n[1], n[2]]]
        
        # 设置初始方向（使用网格索引差值表示）
        grid_direction = None
        
        # 如果没有未访问的活跃邻居，检查是否有两个相对方向的邻居
        if not active_neighbors:
            all_active = [n for n in neighbors if self.grid[n[0], n[1], n[2]] == 1]
            if len(all_active) == 2:
                # 检查两点是否在相对方向上（使用索引差值判断）
                offset1 = all_active[0] - seed_point
                offset2 = all_active[1] - seed_point
                
                # 如果两个偏移量互为相反数，则这两个邻居点在相对方向上
                if (offset1[0] == -offset2[0] and 
                    offset1[1] == -offset2[1] and 
                    offset1[2] == -offset2[2]):
                    # 选择其中一个方向
                    if not self.visited[all_active[0][0], all_active[0][1], all_active[0][2]]:
                        grid_direction = offset1  # 直接使用网格索引差值作为方向
                    else:
                        grid_direction = offset2
        else:
            # 选择第一个未访问的邻居的方向
            grid_direction = active_neighbors[0] - seed_point
        
        # 如果无法确定方向，返回None
        if grid_direction is None:
            return None
        
        # 标记种子点为已访问
        self.visited[seed_point[0], seed_point[1], seed_point[2]] = True
        
        while True:
            # 沿当前方向移动到下一个网格单元
            next_idx = current_idx + grid_direction
            
            # 检查边界条件
            if not np.all(next_idx >= 0) or not np.all(next_idx < self.grid_dims):
                break
                
            # 如果下一个点不是活跃点，结束跟踪
            if self.grid[next_idx[0], next_idx[1], next_idx[2]] == 0:
                break
                
            # 如果下一个点已被访问，可能是连接点
            if self.visited[next_idx[0], next_idx[1], next_idx[2]]:
                # 将其记录为连接点
                next_idx_tuple = tuple(next_idx)
                if next_idx_tuple in self.voxel_centers:
                    node_point = self.voxel_centers[next_idx_tuple]
                else:
                    node_point = self.grid_to_world(next_idx)
                self.nodes.append(node_point)
                break
                
            # 更新当前点
            current_idx = next_idx.copy()
            next_idx_tuple = tuple(current_idx)
            
            # 获取体素中心点
            if next_idx_tuple in self.voxel_centers:
                center_point = self.voxel_centers[next_idx_tuple]
            else:
                center_point = self.grid_to_world(current_idx)
                
            line_points.append(center_point)  # 使用体素中心点
            self.visited[next_idx[0], next_idx[1], next_idx[2]] = True
            
            # 检查周围邻居
            neighbors = self.get_neighbors(next_idx)
            active_neighbors = [n for n in neighbors if self.grid[n[0], n[1], n[2]] == 1 
                              and not self.visited[n[0], n[1], n[2]]]
            
            # 如果有多个活跃邻居，说明是分支点
            if len(active_neighbors) > 1:
                # 创建新节点
                node_idx_tuple = tuple(next_idx)
                if node_idx_tuple in self.voxel_centers:
                    node_coord = self.voxel_centers[node_idx_tuple]
                else:
                    node_coord = self.grid_to_world(next_idx)
                self.nodes.append(node_coord)
                
                # 记录分支点位置，以便后续从该点启动新的线段追踪
                for branch_idx in active_neighbors[1:]:  # 跳过第一个邻居
                    branch_direction = branch_idx - next_idx
                    # 在reconstruct函数中处理这些分支
                    
                break
                
            # 如果只有一个活跃邻居，更新方向
            elif len(active_neighbors) == 1:
                grid_direction = active_neighbors[0] - next_idx
            # 如果没有活跃邻居，结束当前线段
            else:
                break
                
        return line_points
    
    def merge_segments(self):
        """合并相似线段"""
        print("开始合并线段...")
        if not self.segments:
            return
            
        # 计算所有线段的端点
        endpoints = []
        segment_indices = []
        for i, segment in enumerate(self.segments):
            endpoints.append(segment[0])  # 起点
            endpoints.append(segment[-1])  # 终点
            segment_indices.extend([i, i])  # 记录每个端点对应的线段索引
        endpoints = np.array(endpoints)
        
        # 构建KD树
        tree = cKDTree(endpoints)
        
        # 设置搜索半径（基于网格大小）
        search_radius = self.grid_size * 2
        
        # 使用列表副本存储合并后的线段
        merged_segments = self.segments.copy()
        segments_to_remove = set()
        
        merged = True
        iteration = 0
        max_iterations = 10  # 限制最大迭代次数，防止死循环
        
        while merged and iteration < max_iterations:
            merged = False
            iteration += 1
            
            # 重新计算端点和索引
            endpoints = []
            segment_indices = []
            for i, segment in enumerate(merged_segments):
                if i not in segments_to_remove:  # 只处理未被标记为删除的线段
                    endpoints.append(segment[0])  # 起点
                    endpoints.append(segment[-1])  # 终点
                    segment_indices.extend([i, i])  # 记录每个端点对应的线段索引
            
            if len(endpoints) == 0:
                break
                
            endpoints = np.array(endpoints)
            tree = cKDTree(endpoints)
            
            # 使用集合标记已处理的线段
            processed = set()
            
            # 对每个未被删除的线段进行处理
            for i in range(len(merged_segments)):
                if i in processed or i in segments_to_remove:
                    continue
                    
                # 获取当前线段的端点
                current_end = merged_segments[i][-1]
                
                # 在KD树中搜索可能的合并线段
                indices = tree.query_ball_point(current_end, search_radius)
                
                # 过滤出有效的索引
                valid_indices = set()
                for idx in indices:
                    if idx < len(segment_indices):
                        segment_idx = segment_indices[idx]
                        if segment_idx != i and segment_idx not in processed and segment_idx not in segments_to_remove:
                            valid_indices.add(segment_idx)
                
                # 检查每个可能的合并线段
                for j in valid_indices:
                    if j >= len(merged_segments):
                        continue
                        
                    seg1 = merged_segments[i]
                    seg2 = merged_segments[j]
                    
                    # 计算方向向量
                    v1 = seg1[-1] - seg1[0]
                    v2 = seg2[-1] - seg2[0]
                    
                    # 检查向量长度
                    v1_norm = np.linalg.norm(v1)
                    v2_norm = np.linalg.norm(v2)
                    
                    if v1_norm < 1e-6 or v2_norm < 1e-6:
                        continue
                    
                    # 计算相似度
                    similarity = np.abs(np.dot(v1, v2) / (v1_norm * v2_norm))
                    
                    # 计算端点距离
                    dist = np.linalg.norm(seg1[-1] - seg2[0])
                    
                    if similarity > 0.95 and dist < 0.3:
                        # 合并线段（只更新当前线段，标记另一线段为删除）
                        merged_segments[i] = np.vstack((seg1, seg2))
                        segments_to_remove.add(j)
                        merged = True
                        processed.add(j)
                        break
                
                processed.add(i)
        
        # 最后一步：根据标记删除需要删除的线段
        self.segments = [seg for i, seg in enumerate(merged_segments) if i not in segments_to_remove]
        
        print(f"合并完成，剩余 {len(self.segments)} 条线段")
    
    def reconstruct(self):
        """执行重建过程"""
        print("开始重建过程...")
        
        # 使用KNN图生成额外的点
        self.augment_points_with_knn(k=4, num_interpolations=5, max_edge_length=2.0)
        
        # 计算网格大小
        print("计算网格大小...")
        self.compute_grid_size(self.augmented_points)
        
        # 创建体素网格
        self.create_voxel_grid(self.augmented_points)
        
        # 初始化rerun
        rr.init("pylon_reconstruction")
        rr.spawn()
        
        # 记录原始点云和增强点云
        rr.log("original_points", rr.Points3D(self.points))
        rr.log("augmented_points", rr.Points3D(self.augmented_points))
        
        # 记录所有分支点的队列，用于后续处理
        branch_queue = deque()
        
        # 检测种子点并追踪
        print("检测线段...")
        total_voxels = self.grid_dims[0] * self.grid_dims[1] * self.grid_dims[2]
        with tqdm(total=total_voxels, desc="处理体素") as pbar:
            # 第一步：遍历所有体素，寻找初始种子点
            for x in range(self.grid_dims[0]):
                for y in range(self.grid_dims[1]):
                    for z in range(self.grid_dims[2]):
                        if self.grid[x, y, z] == 1 and not self.visited[x, y, z]:
                            voxel_idx = np.array([x, y, z])
                            if self.is_seed_point(voxel_idx):
                                # 从种子点开始追踪线段
                                line_points = self.trace_line(voxel_idx)
                                if line_points and len(line_points) > 1:
                                    self.segments.append(np.array(line_points))
                        pbar.update(1)
            
            # 第二步：处理所有分支点，启动新的线段追踪
            print("处理分支点...")
            
            # 重新扫描未访问的体素，查找新的种子点
            print("查找剩余种子点...")
            for x in range(self.grid_dims[0]):
                for y in range(self.grid_dims[1]):
                    for z in range(self.grid_dims[2]):
                        if self.grid[x, y, z] == 1 and not self.visited[x, y, z]:
                            voxel_idx = np.array([x, y, z])
                            if self.is_seed_point(voxel_idx):
                                # 从新的种子点开始追踪线段
                                line_points = self.trace_line(voxel_idx)
                                if line_points and len(line_points) > 1:
                                    self.segments.append(np.array(line_points))
        
        # print("合并线段...")
        # self.merge_segments()
        
        print("记录结果...")
        if self.segments:
            for i, segment in enumerate(self.segments):
                rr.log(f"segments/segment_{i}", rr.LineStrips3D(segment))
        
        if self.nodes:
            rr.log("nodes", rr.Points3D(np.array(self.nodes)))
        
        print(f"重建完成！共检测到 {len(self.segments)} 条线段，{len(self.nodes)} 个节点")

def main():
    # 读取点云数据
    input_pylon = "./resources/2024_C_44HZ1_14_pylon.ply"
    input_pylon_2 = "./resources/pylon_test.ply"
    plydata = PlyData.read(input_pylon_2)
    points = np.vstack([plydata['vertex']['x'],
                       plydata['vertex']['y'],
                       plydata['vertex']['z']]).T
    
    # 计算点云中心
    center = np.mean(points, axis=0)
    
    # 将点云移动到原点附近
    points = points - center
    
    # 创建重建器并执行重建
    reconstructor = GridSearch(points)
    reconstructor.reconstruct()

if __name__ == "__main__":
    main()
