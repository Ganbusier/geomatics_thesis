#include <iostream>
#include <filesystem>

#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>

#include "custom_ply.h"

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3 Point;
typedef Kernel::Vector_3 Vector;
typedef std::pair<Point, Vector> Point_with_normal;
typedef CGAL::First_of_pair_property_map<Point_with_normal>  Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;
typedef std::vector<Point_with_normal> PointList;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits
<Kernel, PointList, Point_map, Normal_map> Traits;

typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_RANSAC;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Sphere<Traits> Sphere;
typedef CGAL::Shape_detection::Cylinder<Traits> Cylinder;

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

    // For each instance, compute normals and detect shapes using CGAL
    for (const auto& [ins_class, points] : grouped_points) {
        // Construct PCL standard point cloud for normal estimation
        pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : points->points) {
            pcl::PointXYZ xyz_point;
            xyz_point.x = point.x;
            xyz_point.y = point.y;
            xyz_point.z = point.z;
            xyz_cloud->points.push_back(xyz_point);
        }

        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        ne.setInputCloud(xyz_cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(10);
        ne.compute(*normals);

        // Add normal vectors to CustomPoint cloud
        for (size_t i = 0; i < points->points.size(); ++i) {
            points->points[i].normal_x = normals->points[i].normal_x;
            points->points[i].normal_y = normals->points[i].normal_y;
            points->points[i].normal_z = normals->points[i].normal_z;
        }

        PointList cgal_points;
        for(const auto& point : points->points) {
            cgal_points.emplace_back(
                Point(point.x, point.y, point.z), 
                Vector(point.normal_x, point.normal_y, point.normal_z)
            );
        }

        Efficient_RANSAC ransac;
        ransac.set_input(cgal_points);
        ransac.add_shape_factory<Plane>();
        ransac.add_shape_factory<Sphere>();
        ransac.add_shape_factory<Cylinder>();
        ransac.detect();

        // Set different values for each detected shape
        for(const auto& shape : ransac.shapes()) {
            int geo_class_value = -1;
            if(const Plane* plane = dynamic_cast<const Plane*>(shape.get())) {
                geo_class_value = 0; // Plane
                std::cout << "Detected a plane" << std::endl;
            }
            else if(const Sphere* sphere = dynamic_cast<const Sphere*>(shape.get())) {
                geo_class_value = 1; // Sphere
                std::cout << "Detected a sphere" << std::endl;
            }
            else if(const Cylinder* cylinder = dynamic_cast<const Cylinder*>(shape.get())) {
                geo_class_value = 2; // Cylinder
                std::cout << "Detected a cylinder" << std::endl;
            }
            else {
                std::cerr << "Unknown shape detected." << std::endl;
                return -1;
            }
            // set geo_class value for each point
            if(geo_class_value != -1) {
                for(auto idx : shape->indices_of_assigned_points()) {
                    if(idx < points->points.size()) {
                        points->points[idx].geo_class = geo_class_value;
                    }
                }
            }
        }


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