#include <iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>

#include "custom_ply.h"

int main() {
    // declare point cloud
    pcl::PointCloud<CustomPoint>::Ptr cloud(new pcl::PointCloud<CustomPoint>);

    std::string filepath = modifyPLYHeader("../../../resources/pointcloud.ply");
    if (filepath.empty()) {
        std::cerr << "Cannot modify PLY header." << std::endl;
        return -1;
    }

    // load point cloud from modified PLY file
    if (pcl::io::loadPLYFile<CustomPoint>(filepath, *cloud) == -1) {
        PCL_ERROR("Failed to load point cloud from %s.\n", filepath.c_str());
        return -1;
    }

    std::cout << "Successfully loaded: " << cloud->points.size() << " points" << std::endl;

    remove(filepath.c_str());

    // 存储符合sem_class为5的点
    std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr> grouped_points;
    for (const auto& point : cloud->points) {
        if (point.sem_class == 5) {
            // 按照ins_class进行分组
            if (grouped_points.find(point.ins_class) == grouped_points.end()) {
                grouped_points[point.ins_class] = pcl::PointCloud<CustomPoint>::Ptr(new pcl::PointCloud<CustomPoint>);
            }
            grouped_points[point.ins_class]->points.push_back(point);
        }
    }

    // 导出分组后的点云
    for (const auto& [ins_class, points] : grouped_points) {
        std::string output_filename = "./output/output" + std::to_string(ins_class) + ".ply";
        if (pcl::io::savePLYFileBinary(output_filename, *points) == -1) {
            PCL_ERROR("Failed to save filtered points to %s.\n", output_filename.c_str());
            return -1;
        }
        std::cout << "Exported " << points->points.size() << " points to " << output_filename << std::endl;
    }

    return 0;
}
