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
    auto points = cloud->get_vertex_property<vec3>("v:point");
    if (!normals) {
        bool estimate_normals = PointCloudNormals::estimate(cloud);
        if (!estimate_normals) {
            LOG(WARNING) << "No normals found or estimated for point cloud";
            return false;
        }
    }

    // set ransac and parameters
    PrimitivesRansac ransac;
    unsigned int min_support = 30;
    float dist_threshold = 0.02f;
    float bitmap_resolution = 0.01f;
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
        
        // draw cylinders
        for (int i = 0; i < cylinders.size(); i++) {
            auto cylinder = cylinders[i];
            std::vector<int>& vertices = cylinder.vertices;
            std::vector<vec3> cylinder_points;
            for (int& vertex : vertices) {
                cylinder_points.push_back(points[PointCloud::Vertex(vertex)]);
            }

            auto bbox_drawable = new LinesDrawable("bbox");
            const Box3& box = geom::bounding_box<Box3, std::vector<vec3>>(cylinder_points);
            float xmin = box.min_coord(0);
            float xmax = box.max_coord(0);
            float ymin = box.min_coord(1);
            float ymax = box.max_coord(1);
            float zmin = box.min_coord(2);
            float zmax = box.max_coord(2);
            const std::vector<vec3> bbox_points = {
                    vec3(xmin, ymin, zmax), vec3(xmax, ymin, zmax),
                    vec3(xmin, ymax, zmax), vec3(xmax, ymax, zmax),
                    vec3(xmin, ymin, zmin), vec3(xmax, ymin, zmin),
                    vec3(xmin, ymax, zmin), vec3(xmax, ymax, zmin)
            };
            const std::vector<unsigned int> bbox_indices = {
                    0, 1, 2, 3, 4, 5, 6, 7,
                    0, 2, 4, 6, 1, 3, 5, 7,
                    0, 4, 2, 6, 1, 5, 3, 7
            };
            bbox_drawable->update_vertex_buffer(bbox_points);
            bbox_drawable->update_element_buffer(bbox_indices);
            bbox_drawable->set_uniform_coloring(vec4(0.0f, 0.0f, 1.0f, 1.0f));
            bbox_drawable->set_line_width(5.0f);
            viewer->add_drawable(bbox_drawable);

            auto cylinder_drawable = new LinesDrawable("cylinder");
            std::vector<vec3> cylinder_endpoints = {
                cylinder.position,
                cylinder.position + cylinder.direction * 100.0f
            };
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(cylinder.radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->add_drawable(cylinder_drawable);
        }
    }
    return true;
}