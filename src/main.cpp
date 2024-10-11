#include <iostream>
#include <filesystem>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>

#include "custom_ply.h"

int main() {
    // declare point cloud
    pcl::PointCloud<CustomPoint>::Ptr cloud(new pcl::PointCloud<CustomPoint>);

    std::string filepath = modifyPLYHeader("../resources/pointcloud.ply");
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

    // create output folder if not exists
    std::string output_folder = "./output/";
    if (!std::filesystem::exists(output_folder)) {
        std::filesystem::create_directories(output_folder);
    }

    // filter points by semantic class and instance class, in Dalles dataset, semantic class 5 represents power lines
    std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr> grouped_points;
    for (const auto& point : cloud->points) {
        if (point.sem_class == 5) {
            // group points by instance class
            if (grouped_points.find(point.ins_class) == grouped_points.end()) {
                grouped_points[point.ins_class] = pcl::PointCloud<CustomPoint>::Ptr(new pcl::PointCloud<CustomPoint>);
            }
            grouped_points[point.ins_class]->points.push_back(point);
        }
    }
    if (grouped_points.empty()) {
        std::cerr << "No points found in semantic class 5." << std::endl;
        return -1;
    }

    // export filtered points to PLY files
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