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
    loadPLY(cloud, input_file_path, true);

    // filter points by semantic class and instance class
    std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr> grouped_points;
    filterBySemClass(grouped_points, cloud, semantic_class);

    // For each instance, compute normals and detect shapes using CGAL
    for (const auto& [ins_class, points] : grouped_points) {
        ransac_run(points, ins_class);
    }

    // create output folder if not exists
    std::string output_folder = "./output/";
    outputPLY(output_folder, grouped_points);

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