import numpy as np
from sklearn.cluster import KMeans
from sklearn.neighbors import NearestNeighbors
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from plyfile import PlyData

def compute_normals(points, k_neighbors=30):
    """
    计算每个点的局部法向量
    :param points: 输入点云 (N,3)
    :param k_neighbors: 近邻数
    :return: 法向量数组 (N,3)
    """
    # 构建KD树加速近邻搜索
    nbrs = NearestNeighbors(n_neighbors=k_neighbors, algorithm='kd_tree').fit(points)
    _, indices = nbrs.kneighbors(points)
    
    normals = []
    for i in range(len(points)):
        # 获取局部邻域点
        neighborhood = points[indices[i]]
        
        # PCA计算法向量
        cov = np.cov(neighborhood, rowvar=False)
        eigenvalues, eigenvectors = np.linalg.eigh(cov)
        normals.append(eigenvectors[:, np.argmin(eigenvalues)])  # 最小特征值对应法向量
        
    return np.array(normals)

def cluster_normals(normals, n_clusters=5):
    """
    法向量聚类
    :param normals: 法向量数组 (N,3)
    :param n_clusters: 聚类数目
    :return: 聚类标签 (N,)
    """
    # 使用K-means聚类（可根据数据换用DBSCAN）
    kmeans = KMeans(n_clusters=n_clusters)
    labels = kmeans.fit_predict(normals)
    return labels

def fit_plane(points):
    """
    通过PCA拟合平面方程
    :param points: 输入点云 (M,3)
    :return: 平面方程系数 (a,b,c,d), ax+by+cz+d=0
    """
    centroid = np.mean(points, axis=0)
    cov = np.cov(points - centroid, rowvar=False)
    _, eigenvectors = np.linalg.eigh(cov)
    normal = eigenvectors[:, np.argmin(np.linalg.eigvalsh(cov))]  # 法向量
    
    # 计算平面方程
    a, b, c = normal
    d = -np.dot(normal, centroid)
    return (a, b, c, d)

def project_to_plane(points, plane_coeff):
    """
    将3D点投影到平面
    :param points: 输入点云 (N,3)
    :param plane_coeff: 平面方程 (a,b,c,d)
    :return: 投影后的2D点 (N,2)
    """
    a, b, c, d = plane_coeff
    normal = np.array([a, b, c])
    
    # 计算投影矩阵
    I = np.eye(3)
    projection_matrix = I - np.outer(normal, normal) / np.dot(normal, normal)
    
    # 平移点到平面坐标系
    projected_3d = (projection_matrix @ (points + d * normal / np.dot(normal, normal)).T).T
    
    # 构建局部2D坐标系
    u = np.array([normal[1], -normal[0], 0])  # 任意选择与法向量垂直的方向
    u /= np.linalg.norm(u)
    v = np.cross(normal, u)
    
    # 转换为2D坐标
    points_2d = np.column_stack((np.dot(projected_3d, u), np.dot(projected_3d, v)))
    return points_2d

# ---------------------- 使用示例 ----------------------
if __name__ == "__main__":
    # # 生成测试数据（含两个平面）
    # np.random.seed(42)
    # plane1 = np.random.randn(500, 3) * [0.1, 0.1, 1] + [2, 0, 0]
    # plane2 = np.random.randn(500, 3) * [0.1, 1, 0.1] + [0, 2, 0]
    # points = np.vstack((plane1, plane2))

    # read ply data
    input_pylon = "./resources/2024_C_44HZ1_14_pylon.ply"
    ply_data = PlyData.read(input_pylon)
    vertices = ply_data["vertex"]
    vx = vertices["x"]
    vy = vertices["y"]
    vz = vertices["z"]

    # offset point clouds
    minx = np.min(vx)
    miny = np.min(vy)
    minz = np.min(vz)
    vx = [xi - minx for xi in vx]
    vy = [yi - miny for yi in vy]
    vz = [zi - minz for zi in vz]    
    points = np.array([vx, vy, vz]).T
    

    # 1. 计算法向量
    normals = compute_normals(points, k_neighbors=10)
    
    # 2. 法向量聚类
    labels = cluster_normals(normals, n_clusters=20)
    
    # 3. 对每个平面进行投影
    projections = []
    for cluster_id in np.unique(labels):
        cluster_points = points[labels == cluster_id]
        plane_coeff = fit_plane(cluster_points)
        proj_2d = project_to_plane(cluster_points, plane_coeff)
        projections.append(proj_2d)
        
    # 可视化
    fig = plt.figure(figsize=(15,5))
    
    # # 原始点云与法向量
    # ax1 = fig.add_subplot(131, projection='3d')
    # ax1.scatter(points[:,0], points[:,1], points[:,2], c=labels, s=1)
    # ax1.set_title("Original Points with Normals")
    
    # 平面投影结果
    ax2 = fig.add_subplot(132)
    for i, proj in enumerate(projections):
        ax2.scatter(proj[:,0], proj[:,1], s=1)
    ax2.set_title("2D Projections")
    ax2.legend()
    
    # # 法向量方向分布
    # ax3 = fig.add_subplot(133, projection='3d')
    # ax3.quiver(points[:,0], points[:,1], points[:,2], 
    #            normals[:,0], normals[:,1], normals[:,2], length=0.1)
    # ax3.set_title("Normal Vectors")
    
    plt.tight_layout()
    plt.show()