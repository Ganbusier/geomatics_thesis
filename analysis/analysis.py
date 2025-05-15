import numpy as np
import os
import math
import time
import csv
from datetime import datetime

def parse_ply_file(file_path):
    """自定义函数来解析PLY文件，获取顶点和边的信息"""
    vertices = []
    edges = []
    
    try:
        with open(file_path, 'r') as f:
            lines = f.readlines()
        
        # 解析头部信息
        header_end = 0
        vertex_count = 0
        edge_count = 0
        
        for i, line in enumerate(lines):
            line = line.strip()
            if line == "end_header":
                header_end = i
                break
            elif line.startswith("element vertex"):
                vertex_count = int(line.split()[-1])
            elif line.startswith("element edge"):
                edge_count = int(line.split()[-1])
        
        print(f"文件头信息: vertex_count={vertex_count}, edge_count={edge_count}")
        
        # 读取顶点数据
        for i in range(header_end + 1, header_end + 1 + vertex_count):
            if i < len(lines):
                coords = [float(x) for x in lines[i].strip().split()]
                if len(coords) >= 3:  # 确保至少有x,y,z三个坐标
                    vertices.append(np.array(coords[:3]))
        
        # 读取边数据
        for i in range(header_end + 1 + vertex_count, header_end + 1 + vertex_count + edge_count):
            if i < len(lines):
                values = lines[i].strip().split()
                if len(values) >= 3:  # 确保边定义有足够的数据
                    # PLY格式中边通常表示为：count idx1 idx2...
                    # 我们这里假设所有边都是两个顶点之间的
                    vertex_count_in_edge = int(values[0])
                    if vertex_count_in_edge == 2 and len(values) >= 3:
                        idx1 = int(values[1])
                        idx2 = int(values[2])
                        if idx1 < len(vertices) and idx2 < len(vertices):
                            edges.append((vertices[idx1], vertices[idx2]))
        
        print(f"成功从PLY文件解析到 {len(vertices)} 个顶点和 {len(edges)} 条边")
        
    except Exception as e:
        print(f"解析PLY文件时出错: {e}")
        vertices = []
        edges = []
    
    return edges

def export_edges_to_ply(edges, output_file):
    """
    将边导出为PLY文件
    
    参数:
    - edges: 边的列表，每条边为一个包含两个顶点坐标的元组
    - output_file: 输出PLY文件路径
    """
    # 首先，收集所有的顶点，并去重
    unique_vertices = []
    vertex_dict = {}  # 用于映射顶点坐标到索引
    
    for edge in edges:
        for vertex in edge:
            vertex_tuple = tuple(vertex)
            if vertex_tuple not in vertex_dict:
                vertex_dict[vertex_tuple] = len(unique_vertices)
                unique_vertices.append(vertex)
    
    # 创建边的索引列表
    edge_indices = []
    for edge in edges:
        idx1 = vertex_dict[tuple(edge[0])]
        idx2 = vertex_dict[tuple(edge[1])]
        edge_indices.append((idx1, idx2))
    
    # 写入PLY文件
    with open(output_file, 'w') as f:
        # 写入头部
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(unique_vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write(f"element edge {len(edge_indices)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        
        # 写入顶点数据
        for vertex in unique_vertices:
            f.write(f"{vertex[0]} {vertex[1]} {vertex[2]}\n")
        
        # 写入边数据
        for edge in edge_indices:
            f.write(f"2 {edge[0]} {edge[1]}\n")
    
    print(f"已将 {len(edges)} 条边导出到 {output_file}")
    return output_file

def calculate_distance(edge1, edge2):
    """计算edge1的中点到edge2的垂直距离"""
    # 计算edge2的中点
    midpoint = (edge2[0] + edge2[1]) / 2
    
    # 计算edge1的方向向量
    direction = edge1[1] - edge1[0]
    direction_length = np.linalg.norm(direction)
    
    # 防止除以零（如果edge1的两个点重合）
    if direction_length == 0:
        return np.linalg.norm(midpoint - edge1[0])
    
    # 单位化方向向量
    direction = direction / direction_length
    
    # 计算midpoint到edge1起点的向量
    vector_to_line = midpoint - edge1[0]
    
    # 计算向量在方向上的投影长度
    projection_length = np.dot(vector_to_line, direction)
    
    # 如果投影点在线段外部，计算到最近端点的距离
    if projection_length < 0:
        return np.linalg.norm(midpoint - edge1[0])
    elif projection_length > direction_length:
        return np.linalg.norm(midpoint - edge1[1])
    
    # 计算投影点
    projection_point = edge1[0] + projection_length * direction
    
    # 计算midpoint到投影点的距离（即垂直距离）
    return np.linalg.norm(midpoint - projection_point)

def calculate_angle(edge1, edge2):
    """计算两条边之间的角度（以度为单位）"""
    vector1 = edge1[1] - edge1[0]
    vector2 = edge2[1] - edge2[0]
    
    # 归一化向量
    vector1_norm = np.linalg.norm(vector1)
    vector2_norm = np.linalg.norm(vector2)
    
    # 防止除以零
    if vector1_norm == 0 or vector2_norm == 0:
        return 0
    
    vector1 = vector1 / vector1_norm
    vector2 = vector2 / vector2_norm
    
    # 计算点积
    dot_product = np.clip(np.dot(vector1, vector2), -1.0, 1.0)
    
    # 计算角度（以度为单位）
    angle_rad = np.arccos(dot_product)
    angle_deg = np.degrees(angle_rad)
    
    # 返回较小的角度（0-90度范围）
    return min(angle_deg, 180 - angle_deg)

def print_progress_bar(iteration, total, prefix='', suffix='', length=50, fill='█'):
    """
    打印进度条
    
    参数:
    - iteration: 当前迭代次数
    - total: 总迭代次数
    - prefix: 前缀字符串
    - suffix: 后缀字符串
    - length: 进度条长度
    - fill: 进度条填充字符
    """
    percent = ("{0:.1f}").format(100 * (iteration / float(total)))
    filled_length = int(length * iteration // total)
    bar = fill * filled_length + '-' * (length - filled_length)
    print(f'\r{prefix} |{bar}| {percent}% {suffix}', end='\r')
    # 打印新行
    if iteration == total:
        print()

def save_results_to_csv(results, gt_file, ext_file, distance_threshold, angle_threshold, output_dir="analysis/output"):
    """
    将结果保存到CSV文件
    
    参数:
    - results: 结果列表，每个元素为(gt_idx, distance_matches, angle_matches, avg_dist, std_dist, avg_angle, std_angle)
    - gt_file: 地面真值文件路径
    - ext_file: 提取边文件路径
    - distance_threshold: 距离阈值
    - angle_threshold: 角度阈值
    - output_dir: 输出目录
    """
    # 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)
    
    # 提取文件名用于CSV文件命名
    ext_name = os.path.basename(os.path.dirname(ext_file))
    
    # 构建输出文件名
    output_file = os.path.join(output_dir, f"{ext_name}.csv")
    
    # 写入CSV文件
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        
        # 写入头部信息
        writer.writerow(['Analysis Info', f'Distance Threshold: {distance_threshold}', f'Angle Threshold: {angle_threshold}'])
        writer.writerow(['GT File', gt_file])
        writer.writerow(['Extracted Edges File', ext_file])
        writer.writerow([])
        
        # 写入列标题
        writer.writerow([
            'GT Edge Index', 
            'Edges Within Distance Threshold', 
            'Edges Within Angle Threshold',
            'Avg Distance',
            'Std Distance',
            'Avg Angle',
            'Std Angle'
        ])
        
        # 写入每条GT边的匹配情况
        for gt_idx, dist_matches, angle_matches, avg_dist, std_dist, avg_angle, std_angle in results:
            writer.writerow([
                gt_idx, 
                dist_matches, 
                angle_matches,
                f"{avg_dist:.6f}" if avg_dist is not None else "N/A",
                f"{std_dist:.6f}" if std_dist is not None else "N/A",
                f"{avg_angle:.6f}" if avg_angle is not None else "N/A",
                f"{std_angle:.6f}" if std_angle is not None else "N/A"
            ])
        
        # 写入汇总信息
        writer.writerow([])
        total_distance_matches = sum(dist for _, dist, _, _, _, _, _ in results)
        total_angle_matches = sum(angle for _, _, angle, _, _, _, _ in results)
        avg_distance_matches = total_distance_matches / len(results) if results else 0
        avg_angle_matches = total_angle_matches / len(results) if results else 0
        
        # 计算所有GT边的平均统计值
        valid_avg_distances = [avg_dist for _, _, _, avg_dist, _, _, _ in results if avg_dist is not None]
        valid_std_distances = [std_dist for _, _, _, _, std_dist, _, _ in results if std_dist is not None]
        valid_avg_angles = [avg_angle for _, _, _, _, _, avg_angle, _ in results if avg_angle is not None]
        valid_std_angles = [std_angle for _, _, _, _, _, _, std_angle in results if std_angle is not None]
        
        overall_avg_distance = sum(valid_avg_distances) / len(valid_avg_distances) if valid_avg_distances else 0
        overall_avg_std_distance = sum(valid_std_distances) / len(valid_std_distances) if valid_std_distances else 0
        overall_avg_angle = sum(valid_avg_angles) / len(valid_avg_angles) if valid_avg_angles else 0
        overall_avg_std_angle = sum(valid_std_angles) / len(valid_std_angles) if valid_std_angles else 0
        
        writer.writerow(['Summary', '', '', '', '', '', ''])
        writer.writerow(['Total Edges Within Distance Threshold', total_distance_matches, '', '', '', '', ''])
        writer.writerow(['Total Edges Within Angle Threshold', total_angle_matches, '', '', '', '', ''])
        writer.writerow(['Avg Edges Within Distance Threshold Per GT Edge', f"{avg_distance_matches:.6f}", '', '', '', '', ''])
        writer.writerow(['Avg Edges Within Angle Threshold Per GT Edge', f"{avg_angle_matches:.6f}", '', '', '', '', ''])
        writer.writerow(['Average of All Avg Distances', f"{overall_avg_distance:.6f}", '', '', '', '', ''])
        writer.writerow(['Average of All Std Distances', f"{overall_avg_std_distance:.6f}", '', '', '', '', ''])
        writer.writerow(['Average of All Avg Angles', f"{overall_avg_angle:.6f}", '', '', '', '', ''])
        writer.writerow(['Average of All Std Angles', f"{overall_avg_std_angle:.6f}", '', '', '', '', ''])
    
    print(f"分析结果已保存至: {output_file}")
    return output_file

def main():
    # 文件路径
    ground_truth_file = os.path.join("analysis", "groundTruth", "pylon1_gt.ply")
    extracted_edges_file = os.path.join("analysis", "data", "pylon1", "preservedEdges.ply")
    
    # 阈值设置
    distance_threshold = 0.1  # 距离阈值
    angle_threshold = 20.0   # 角度阈值（度）
    
    # 输出目录
    output_dir = os.path.join("analysis", "output")
    os.makedirs(output_dir, exist_ok=True)
    
    print(f"加载地面真值边: {ground_truth_file}")
    ground_truth_edges = parse_ply_file(ground_truth_file)
    print(f"加载提取的边: {extracted_edges_file}")
    extracted_edges = parse_ply_file(extracted_edges_file)
    
    print(f"地面真值边数量: {len(ground_truth_edges)}")
    print(f"提取的边数量: {len(extracted_edges)}")
    
    # 统计结果
    start_time = time.time()
    total_distance_matched = 0       # 符合距离阈值的边总数
    total_angle_matched = 0          # 符合角度阈值的边总数
    distance_calculations = 0
    
    # 记录每条GT边的匹配情况
    gt_edge_results = []
    
    # 存储所有符合条件的边，用于导出为PLY文件
    matched_edges = []
    matched_edges_per_gt = {}  # 每条GT边对应的匹配边
    
    # 计算总对比次数，用于显示进度
    total_comparisons = len(ground_truth_edges)
    print(f"\n开始分析，总共 {total_comparisons} 条地面真值边需要处理...")
    
    # 分析每条地面真值边
    for gt_idx, gt_edge in enumerate(ground_truth_edges):
        # 显示进度条
        print_progress_bar(gt_idx + 1, total_comparisons, prefix='处理进度:', 
                          suffix=f'({gt_idx + 1}/{total_comparisons})', length=40)
        
        # 找到符合距离阈值的提取边
        distance_matches = 0
        angle_matches = 0
        
        # 存储每条GT边的所有符合距离阈值的距离和角度
        distances = []
        angles = []
        
        # 存储当前GT边的匹配边
        current_gt_matches = []
        
        for ext_idx, ext_edge in enumerate(extracted_edges):
            distance_calculations += 1
            dist = calculate_distance(gt_edge, ext_edge)
            angle = calculate_angle(gt_edge, ext_edge)
            
            if dist <= distance_threshold and angle <= angle_threshold:
                distance_matches += 1
                angle_matches += 1
                distances.append(dist)
                angles.append(angle)
                
                # 添加到匹配边列表
                matched_edges.append(ext_edge)
                current_gt_matches.append(ext_edge)
        
        # 记录当前GT边的匹配边
        if current_gt_matches:
            matched_edges_per_gt[gt_idx] = current_gt_matches
        
        # 计算统计信息
        avg_dist = np.mean(distances) if distances else None
        std_dist = np.std(distances) if len(distances) > 1 else None
        avg_angle = np.mean(angles) if angles else None
        std_angle = np.std(angles) if len(angles) > 1 else None
        
        # 累加每条GT边的匹配结果
        total_distance_matched += distance_matches
        total_angle_matched += angle_matches
        
        # 保存每条GT边的匹配情况
        gt_edge_results.append((gt_idx, distance_matches, angle_matches, avg_dist, std_dist, avg_angle, std_angle))
    
    end_time = time.time()
    total_time = end_time - start_time
    
    # 计算平均每条GT边的匹配情况
    avg_distance_matches = total_distance_matched / len(ground_truth_edges) if ground_truth_edges else 0
    avg_angle_matches = total_angle_matched / len(ground_truth_edges) if ground_truth_edges else 0
    
    # 保存结果到CSV文件
    csv_file = save_results_to_csv(
        gt_edge_results, 
        ground_truth_file, 
        extracted_edges_file, 
        distance_threshold, 
        angle_threshold,
        output_dir
    )
    
    # 导出所有匹配的边到PLY文件
    ext_name = os.path.basename(os.path.dirname(extracted_edges_file))
    
    # 导出所有匹配的边
    all_matched_ply_file = os.path.join(output_dir, f"{ext_name}_matchedEdges.ply")
    export_edges_to_ply(matched_edges, all_matched_ply_file)
    
    # 输出基本统计信息
    print("\n" + "=" * 50)
    print(f"===== 分析结果摘要 =====")
    print(f"执行总时间: {total_time:.2f} 秒")
    print(f"距离计算次数: {distance_calculations}")
    print(f"符合距离阈值的边总数: {total_distance_matched}")
    print(f"符合角度阈值的边总数: {total_angle_matched}")
    print(f"平均每条GT边的距离匹配数: {avg_distance_matches:.2f}")
    print(f"平均每条GT边的角度匹配数: {avg_angle_matches:.2f}")
    print(f"详细分析结果已保存至: {csv_file}")
    print(f"所有匹配的边已导出至: {all_matched_ply_file}")
    print("=" * 50)

if __name__ == "__main__":
    main()
