#include <filesystem>
#include <iostream>

// #include "custom_point.h"
// #include "ransac.h"

#include <easy3d/viewer/viewer.h>
#include <easy3d/core/model.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/algo/point_cloud_ransac.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/util/resource.h>
#include <easy3d/util/initializer.h>


using namespace easy3d;

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path> <semantic_class>" << std::endl;
        return -1;
    }

    // read semantic class from command line argument
    std::string input_file_path = argv[1];
    int semantic_class = std::stoi(argv[2]);

    // // declare point cloud
    // pcl::PointCloud<CustomPoint>::Ptr cloud(new pcl::PointCloud<CustomPoint>);
    // loadPLY(cloud, input_file_path, true);

    // // filter points by semantic class and instance class
    // std::unordered_map<int, pcl::PointCloud<CustomPoint>::Ptr> grouped_points;
    // filterBySemClass(grouped_points, cloud, semantic_class);
    // std::cout << "Number of instances: " << grouped_points.size() << std::endl;
    
    // // For each instance, compute normals and detect shapes using CGAL
    // for (const auto& [ins_class, points] : grouped_points) {
    //     ransac_run(points, ins_class);
    // }

    // // create output folder if not exists
    // std::string output_folder = "./output/";
    // outputPLY(output_folder, grouped_points);

    initialize();

    Viewer viewer("Geomatics Thesis");
    Model* model = viewer.add_model(input_file_path, true);

    // read point cloud
    auto cloud = dynamic_cast<PointCloud *>(model);
    if (!cloud) {
        LOG(ERROR) << "Failed to load point cloud from " << input_file_path;
        return EXIT_FAILURE;
    }

    std::cout << "Number of points: " << cloud->n_vertices() << std::endl;

    // get vertex properties
    auto points = cloud->get_vertex_property<vec3>("v:point");
    auto semantics = cloud->get_vertex_property<int>("v:sem_class");

    // estimate normals
    if (PointCloudNormals::estimate(cloud)) {
        auto normals = cloud->get_vertex_property<vec3>("v:normal");
    }

    return viewer.run();
}