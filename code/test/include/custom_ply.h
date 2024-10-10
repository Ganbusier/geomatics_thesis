#ifndef CUSTOM_PLY_H
#define CUSTOM_PLY_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h> 
#include <string> 

std::string modifyPLYHeader(const std::string &filepath);

struct CustomPoint {
    PCL_ADD_POINT4D;  // keep compatible with pcl::PointXYZ
    int intensity;
    int sem_class;
    int ins_class;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(CustomPoint,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (int, intensity, intensity)
                                  (int, sem_class, sem_class)
                                  (int, ins_class, ins_class))
#endif // CUSTOM_PLY_H