#ifndef RANSAC_H
#define RANSAC_H

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

#include <iostream>

#include "custom_ply.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> Point_with_normal;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef std::vector<Point_with_normal> PointList;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, PointList, Point_map, Normal_map>
    Traits;

typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_RANSAC;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Sphere<Traits> Sphere;
typedef CGAL::Shape_detection::Cylinder<Traits> Cylinder;

void ransac_run(const std::shared_ptr<pcl::PointCloud<CustomPoint>>& points,
                const int& ins_class);

#endif  // RANSAC_H