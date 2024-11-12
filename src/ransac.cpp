#include "../include/ransac.h"

void ransac_run(const std::shared_ptr<pcl::PointCloud<CustomPoint>>& points,
                const int& ins_class) {
    
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
    for (const auto& point : points->points) {
        cgal_points.emplace_back(Point(point.x, point.y, point.z),
                                    Vector(point.normal_x, point.normal_y, point.normal_z));
    }

    Efficient_RANSAC ransac;
    ransac.set_input(cgal_points);
    ransac.add_shape_factory<Plane>();
    ransac.add_shape_factory<Sphere>();
    ransac.add_shape_factory<Cylinder>();

    Efficient_RANSAC::Parameters parameters;
    parameters.epsilon = 0.5f;
    parameters.normal_threshold = 0.9f;
    parameters.cluster_epsilon = 0.01f;
    parameters.min_points = 10;
    parameters.probability = 0.05f;

    ransac.detect(parameters);

    // Set different values for each detected shape, if not detected, set to -1
    if (ransac.shapes().empty()) {
        for (auto& point : points->points) {
            point.geo_class = -1;
        }
        std::cerr << "Instance " << ins_class << ": No shapes detected." << std::endl;
    } else {
        int shape_idx = 0;
        int plane_count = 0;
        int sphere_count = 0;
        int cylinder_count = 0;
        for (const auto& shape : ransac.shapes()) {
            int geo_class_value = -1;
            if (const Plane* plane = dynamic_cast<const Plane*>(shape.get())) {
                geo_class_value = 0;  // Plane
                plane_count++;
            } else if (const Sphere* sphere = dynamic_cast<const Sphere*>(shape.get())) {
                geo_class_value = 1;  // Sphere
                sphere_count++;
            } else if (const Cylinder* cylinder = dynamic_cast<const Cylinder*>(shape.get())) {
                geo_class_value = 2;  // Cylinder
                cylinder_count++;
            } else {
                std::cerr << "Instance " << ins_class << ": Unknown shape detected." << std::endl;
                return;
            }
            // set geo_class value for each point
            for (auto idx : shape->indices_of_assigned_points()) {
                if (idx < points->points.size()) {
                    points->points[idx].geo_class = geo_class_value;
                    points->points[idx].shape_idx = shape_idx;
                }
            }
            shape_idx++;
        }
        std::cout << "Instance " << ins_class << ": Detected " << plane_count << " planes, "
                  << sphere_count << " spheres, and " << cylinder_count << " cylinders."
                  << std::endl;
    }
}