#include "custom_ply.h"

// this function is designed for Dalles dataset specifically, which modifies
// the PLY header to make it compatible with PCL library and returns the temporary file path
std::string modifyPLYHeader(const std::string& filepath) {
    std::ifstream infile(filepath);
    std::string temp_filepath = "/tmp/temp_pointcloud.ply";
    std::ofstream outfile(temp_filepath);

    if (!infile || !outfile) {
        std::cerr << "Failed to open file: " << filepath << std::endl;
        return "";
    }

    std::string line;

    // change "testing" to "vertex"
    while (std::getline(infile, line)) {
        if (line.find("element testing") != std::string::npos) {
            line.replace(line.find("testing"), 7, "vertex");
        }
        outfile << line << std::endl;
    }

    infile.close();
    outfile.close();

    return temp_filepath;
}

void loadPLY(pcl::PointCloud<CustomPoint>::Ptr& cloud, std::string input_file_path,
             bool modify_header) {
    std::string filepath = input_file_path;
    if (modify_header) {
        filepath = modifyPLYHeader(input_file_path);
    }

    if (filepath.empty()) {
        std::cerr << "Cannot modify PLY header." << std::endl;
        return;
    }

    // load point cloud from modified PLY file
    if (pcl::io::loadPLYFile<CustomPoint>(filepath, *cloud) == -1) {
        PCL_ERROR("Failed to load point cloud from %s.\n", filepath.c_str());
        return;
    }

    std::cout << "Successfully loaded: " << cloud->points.size() << " points" << std::endl;

    if (modify_header) remove(filepath.c_str());
}

void filterBySemClass(std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr>& grouped_points,
                      pcl::PointCloud<CustomPoint>::Ptr& cloud, const int sem_class) {
    for (const auto& point : cloud->points) {
        if (point.sem_class == sem_class) {
            // group points by instance class
            if (grouped_points.find(point.ins_class) == grouped_points.end()) {
                grouped_points[point.ins_class] =
                    pcl::PointCloud<CustomPoint>::Ptr(new pcl::PointCloud<CustomPoint>);
            }
            grouped_points[point.ins_class]->points.push_back(point);
        }
    }
    if (grouped_points.empty()) {
        std::cerr << "No points found in semantic class " << sem_class << std::endl;
        return;
    }
}
