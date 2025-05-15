import rerun as rr
import numpy as np
import os
import argparse

def parse_ply_file(file_path):
    """解析PLY文件，获取顶点和边的信息，可自动区分点云文件和边文件"""
    vertices = []
    edges = []
    
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # 解析头部信息
        header_end = 0
        vertex_count = 0
        edge_count = 0
        is_point_cloud_only = True  # 默认当作点云文件
        
        for i, line in enumerate(lines):
            line = line.strip()
            if line == "end_header":
                header_end = i
                break
            elif line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            elif line.startswith("element edge"):
                edge_count = int(line.split()[-1])
                is_point_cloud_only = False  # 有edge定义，不是纯点云文件
        
        # 读取顶点数据
        for i in range(header_end + 1, header_end + 1 + vertex_count):
            if i < len(lines):
                coords = [float(x) for x in lines[i].strip().split()]
                if len(coords) >= 3:  # 确保至少有x,y,z三个坐标
                    vertices.append(np.array(coords[:3]))
        
        # 读取边数据（如果有）
        if not is_point_cloud_only and edge_count > 0:
            for i in range(header_end + 1 + vertex_count, header_end + 1 + vertex_count + edge_count):
                if i < len(lines):
                    values = lines[i].strip().split()
                    if len(values) >= 3:  # 确保边定义有足够的数据
                        vertex_count_in_edge = int(values[0])
                        if vertex_count_in_edge == 2 and len(values) >= 3:
                            idx1 = int(values[1])
                            idx2 = int(values[2])
                            if idx1 < len(vertices) and idx2 < len(vertices):
                                edges.append((vertices[idx1], vertices[idx2]))
        
        file_type = "点云和边文件" if edges else "纯点云文件"
        print(f"从 {os.path.basename(file_path)} ({file_type}) 解析到 {len(vertices)} 个顶点和 {len(edges)} 条边")
        
    except Exception as e:
        print(f"解析PLY文件时出错: {e}")
        vertices = []
        edges = []
    
    return vertices, edges

def visualize_powerline(name, point_cloud_file, gt_file, match_edges_file=None, 
                       preserved_edges_file=None, removed_edges_file=None):
    """可视化电力线数据"""
    # 初始化rerun
    rr.init(f"powerline_analysis_{name}", spawn=True)
    
    # 解析点云文件
    if os.path.exists(point_cloud_file):
        points, _ = parse_ply_file(point_cloud_file)
        if points:
            # 可视化点云
            rr.log("point_cloud", rr.Points3D(points, radii=0.05, colors=(200, 200, 200, 100)))
            print(f"已加载点云: {len(points)}个点")
    else:
        print(f"警告: 点云文件 {point_cloud_file} 不存在")
    
    # 解析ground truth边文件
    if os.path.exists(gt_file):
        _, gt_edges = parse_ply_file(gt_file)
        if gt_edges:
            # 可视化ground truth边（绿色）
            rr.log("ground_truth_edges", rr.LineStrips3D(gt_edges, colors=(0, 255, 0, 255), radii=0.05))
            print(f"已加载地面真值边: {len(gt_edges)}条")
    else:
        print(f"警告: 地面真值边文件 {gt_file} 不存在")
    
    # 解析匹配的边文件
    if match_edges_file and os.path.exists(match_edges_file):
        _, match_edges = parse_ply_file(match_edges_file)
        if match_edges:
            # 可视化匹配的边（蓝色）
            rr.log("matched_edges", rr.LineStrips3D(match_edges, colors=(0, 0, 255, 255), radii=0.05))
            print(f"已加载匹配的边: {len(match_edges)}条")
    
    # 解析保留的边文件
    if preserved_edges_file and os.path.exists(preserved_edges_file):
        _, preserved_edges = parse_ply_file(preserved_edges_file)
        if preserved_edges:
            # 可视化保留的边（黄色）
            rr.log("preserved_edges", rr.LineStrips3D(preserved_edges, colors=(255, 255, 0, 255), radii=0.05))
            print(f"已加载保留的边: {len(preserved_edges)}条")
    
    # 解析移除的边文件
    if removed_edges_file and os.path.exists(removed_edges_file):
        _, removed_edges = parse_ply_file(removed_edges_file)
        if removed_edges:
            # 可视化移除的边（红色）
            rr.log("removed_edges", rr.LineStrips3D(removed_edges, colors=(255, 0, 0, 255), radii=0.05))
            print(f"已加载移除的边: {len(removed_edges)}条")

def main():
    parser = argparse.ArgumentParser(description='可视化电力线数据')
    parser.add_argument('--name', type=str, default='1powerline', help='数据集名称')
    parser.add_argument('--point_cloud', type=str, help='点云文件路径')
    parser.add_argument('--gt', type=str, help='Ground Truth边文件路径')
    parser.add_argument('--match', type=str, help='匹配的边文件路径')
    parser.add_argument('--preserved', type=str, help='保留的边文件路径')
    parser.add_argument('--removed', type=str, help='移除的边文件路径')
    
    args = parser.parse_args()
    
    # 如果没有提供参数，使用默认路径
    name = args.name
    point_cloud_file = args.point_cloud or f"resources/{name}_pt.ply"
    gt_file = args.gt or f"analysis/groundTruth/{name}_gt.ply"
    match_edges_file = args.match or f"analysis/output/{name}_matchedEdges.ply"
    preserved_edges_file = args.preserved or f"analysis/data/{name}/preservedEdges.ply"
    removed_edges_file = args.removed or f"analysis/data/{name}/removedEdges.ply"
    
    print(f"正在可视化 {name} 数据集")
    print(f"点云文件: {point_cloud_file}")
    print(f"Ground Truth边文件: {gt_file}")
    print(f"匹配的边文件: {match_edges_file}")
    print(f"保留的边文件: {preserved_edges_file}")
    print(f"移除的边文件: {removed_edges_file}")
    
    visualize_powerline(name, point_cloud_file, gt_file, match_edges_file, 
                       preserved_edges_file, removed_edges_file)
    
    print("可视化完成，请在rerun查看器中查看结果")

if __name__ == "__main__":
    main()
