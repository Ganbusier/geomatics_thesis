#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/property_map.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <iostream>

#include "custom_ply.h"
#include "ransac.h"

// #include <rerun.hpp>
// #include <rerun/demo_utils.hpp>

// using namespace rerun::demo;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path> <semantic_class>" << std::endl;
        return -1;
    }

    // read semantic class from command line argument
    std::string input_file_path = argv[1];
    int semantic_class = std::stoi(argv[2]);

    // declare point cloud
    pcl::PointCloud<CustomPoint>::Ptr cloud(new pcl::PointCloud<CustomPoint>);

    std::string filepath = modifyPLYHeader(input_file_path);
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

    // filter points by semantic class and instance class
    std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr> grouped_points;
    for (const auto& point : cloud->points) {
        if (point.sem_class == semantic_class) {
            // group points by instance class
            if (grouped_points.find(point.ins_class) == grouped_points.end()) {
                grouped_points[point.ins_class] =
                    pcl::PointCloud<CustomPoint>::Ptr(new pcl::PointCloud<CustomPoint>);
            }
            grouped_points[point.ins_class]->points.push_back(point);
        }
    }
    if (grouped_points.empty()) {
        std::cerr << "No points found in semantic class " << semantic_class << std::endl;
        return -1;
    }

    // For each instance, compute normals and detect shapes using CGAL
    for (const auto& [ins_class, points] : grouped_points) {
        ransac_run(points, ins_class);
    }

    // export filtered points to PLY files
    for (const auto& [ins_class, points] : grouped_points) {
        std::string output_filename = "./output/output" + std::to_string(ins_class) + ".ply";
        if (pcl::io::savePLYFileBinary(output_filename, *points) == -1) {
            PCL_ERROR("Failed to save filtered points to %s.\n", output_filename.c_str());
            return -1;
        }
        std::cout << "Exported " << points->points.size() << " points to " << output_filename
                  << std::endl;
    }

    // create rerun recording
    // const auto rec = rerun::RecordingStream("thesis");
    // rec.spawn().exit_on_failure();

    // std::vector<rerun::Position3D> rec_points;
    // std::vector<rerun::Color> rec_colors;

    // for(const auto& point : grouped_points.at(0)->points) {
    //     rec_points.emplace_back(point.x, point.y, point.z);
    //     if(point.geo_class == 0) rec_colors.emplace_back(255, 0, 0); // Plane
    //     else if(point.geo_class == 1) rec_colors.emplace_back(0, 255, 0); // Sphere
    //     else if(point.geo_class == 2) rec_colors.emplace_back(0, 0, 255); // Cylinder
    //     else rec_colors.emplace_back(255, 255, 255); // Unknown
    // }

    // rec.log("my_points", rerun::Points3D(rec_points).with_colors(rec_colors).with_radii({0.5f}));

    return 0;
}