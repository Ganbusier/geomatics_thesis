#ifndef CUSTOM_PLY_H
#define CUSTOM_PLY_H

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <iostream>
#include <string>

struct CustomPoint {
    PCL_ADD_POINT4D;  // keep compatible with pcl::PointXYZ
    float normal_x;
    float normal_y;
    float normal_z;
    int intensity;
    int sem_class;
    int ins_class;
    int geo_class;
    int shape_idx;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(
    CustomPoint,
    (float, x, x)(float, y, y)(float, z, z)(float, normal_x, normal_x)(float, normal_y, normal_y)(
        float, normal_z, normal_z)(int, intensity, intensity)(int, sem_class, sem_class)(
        int, ins_class, ins_class)(int, geo_class, geo_class)(int, shape_idx, shape_idx))

std::string modifyPLYHeader(const std::string& filepath);

void filterBySemClass(std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr>& grouped_points,
                      pcl::PointCloud<CustomPoint>::Ptr& cloud, const int sem_class);

void loadPLY(pcl::PointCloud<CustomPoint>::Ptr& cloud, std::string input_file_path,
             bool modify_header = false);

#endif  // CUSTOM_PLY_H