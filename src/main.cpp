#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/algo/point_cloud_ransac.h>
#include <easy3d/core/model.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/initializer.h>
#include <easy3d/util/resource.h>
#include <easy3d/viewer/viewer.h>

#include <filesystem>
#include <iostream>

using namespace easy3d;

bool extract_cylinders(Viewer* viewer, Model* model);

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <input_file_path>" << std::endl;
        return -1;
    }

    std::string input_file_path = argv[1];

    initialize(true);
    LOG(INFO) << "Easy3D initialized";

    Viewer viewer("Geomatics Thesis");
    Model* model = viewer.add_model(input_file_path, true);

    // set up rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_point_size(5.0f);

    // usage
    viewer.set_usage("'Ctrl + e': extract cylinders");
    viewer.bind(extract_cylinders, model, Viewer::KEY_E, Viewer::MODIF_CTRL);
    viewer.fit_screen();

    return viewer.run();
}

bool extract_cylinders(Viewer* viewer, Model* model) {
    if (!viewer || !model) {
        return false;
    }

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    if (!normals) {
        bool estimate_normals = PointCloudNormals::estimate(cloud);
        if (!estimate_normals) {
            LOG(WARNING) << "No normals found or estimated for point cloud";
            return false;
        }
    }

    // set ransac and parameters
    PrimitivesRansac ransac;
    unsigned int min_support = 1000;
    float dist_threshold = 0.005f;
    float bitmap_resolution = 0.02f;
    float normal_threshold = 0.8f;
    float overlook_probability = 0.001f;

    // detect cylinders
    ransac.add_primitive_type(PrimitivesRansac::CYLINDER);
    int num_cylinders = ransac.detect(cloud, min_support, dist_threshold, bitmap_resolution,
                                      normal_threshold, overlook_probability);
    if (num_cylinders > 0) {
        LOG(INFO) << "Detected " << num_cylinders << " cylinders";
        auto cylinders = ransac.get_cylinders();
        auto segments = cloud->vertex_property<int>("v:primitive_index");
        const std::string color_name = "v:color-segments";
        auto coloring = cloud->vertex_property<vec3>(color_name, vec3(0.0f));
        Renderer::color_from_segmentation(cloud, segments, coloring);

        auto drawable = cloud->renderer()->get_points_drawable("vertices");
        drawable->set_property_coloring(State::VERTEX, color_name);
        drawable->update();
        viewer->update();

        for (int i = 0; i < cylinders.size(); i++) {
            auto cylinder = cylinders[i];
            auto cylinder_drawable = model->renderer()->add_lines_drawable("cylinder" + std::to_string(i));
            std::vector<vec3> cylinder_points = {
                cylinder.position,
                cylinder.position + cylinder.direction
            };
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_points);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_line_width(cylinder.radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->update();
        }
    }
    return true;
}