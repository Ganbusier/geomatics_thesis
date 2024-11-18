#ifndef CUSTOM_POINT_H
#define CUSTOM_POINT_H

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

struct CustomPoint {
    PCL_ADD_POINT4D;  // keep compatible with pcl::PointXYZ
    float normal_x = 0.0f;
    float normal_y = 0.0f;
    float normal_z = 0.0f;
    int intensity = 0;
    int sem_class = -1; // for dalles dataset
    int semantics = -1; // for AHN dataset
    int ins_class = -1;
    int geo_class = -1;
    int shape_idx = -1;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CustomPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(
        float, normal_z, normal_z)(int, intensity, intensity)(int, sem_class, sem_class)(
        int, ins_class, ins_class)(int, geo_class, geo_class)(int, shape_idx, shape_idx))

std::string modifyPLYHeader(const std::string filepath);

void filterBySemClass(std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr>& grouped_points,
                      const pcl::PointCloud<CustomPoint>::Ptr& cloud, const int sem_class);

void loadPLY(pcl::PointCloud<CustomPoint>::Ptr& cloud, std::string input_file_path,
             bool modify_header = false);

void outputPLY(std::string output_folder,
               std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr>& grouped_points);
#endif  // CUSTOM_POINT_H