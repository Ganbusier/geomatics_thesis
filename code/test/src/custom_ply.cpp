#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>


// this function is designed for Dalles dataset specifically, which modifies 
// the PLY header to make it compatible with PCL library and returns the temporary file path
std::string modifyPLYHeader(const std::string &filepath) {
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