#include <filesystem>
#include <iostream>

#include "custom_point.h"
#include "ransac.h"


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
    std::cout << "Number of instances: " << grouped_points.size() << std::endl;
    
    // For each instance, compute normals and detect shapes using CGAL
    for (const auto& [ins_class, points] : grouped_points) {
        ransac_run(points, ins_class);
    }

    // create output folder if not exists
    std::string output_folder = "./output/";
    outputPLY(output_folder, grouped_points);

    return 0;
}