#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/property_map.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/algo/point_cloud_ransac.h>
#include <easy3d/core/model.h>
#include <easy3d/core/point_cloud.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/renderer/drawable_lines.h>
#include <easy3d/renderer/drawable_points.h>
#include <easy3d/renderer/renderer.h>
#include <easy3d/util/initializer.h>
#include <easy3d/util/resource.h>
#include <easy3d/viewer/viewer.h>

#include <filesystem>
#include <iostream>

// Type declarations.
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::FT FT;
typedef std::pair<Kernel::Point_3, Kernel::Vector_3> Point_with_normal;
typedef std::vector<Point_with_normal> Pwn_vector;
typedef CGAL::First_of_pair_property_map<Point_with_normal> Point_map;
typedef CGAL::Second_of_pair_property_map<Point_with_normal> Normal_map;

typedef CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map>
    Traits;
typedef CGAL::Shape_detection::Efficient_RANSAC<Traits> Efficient_ransac;
typedef CGAL::Shape_detection::Cylinder<Traits> Cylinder;
typedef CGAL::Shape_detection::Plane<Traits> Plane;
typedef CGAL::Shape_detection::Sphere<Traits> Sphere;

using namespace easy3d;

bool run_easy3d_ransac(Viewer* viewer, Model* model);
bool run_cgal_ransac(Viewer* viewer, Model* model);
bool estimate_normals(Viewer* viewer, Model* model);

std::vector<Drawable*> drawables;  // store drawables added to the viewer
int k_neighbors = 50;              // k-nearest neighbors for normal estimation

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
    viewer.set_usage("'Ctrl + n': estimate normals");
    viewer.set_usage("'Ctrl + e': run CGAL RANSAC");
    viewer.set_usage("'Shift + e': run Easy3D RANSAC");
    viewer.bind(run_easy3d_ransac, model, Viewer::KEY_E, Viewer::MODIF_SHIFT);
    viewer.bind(run_cgal_ransac, model, Viewer::KEY_E, Viewer::MODIF_CTRL);
    viewer.bind(estimate_normals, model, Viewer::KEY_N, Viewer::MODIF_CTRL);

    // fit screen
    viewer.fit_screen();

    return viewer.run();
}

bool estimate_normals(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;
    auto cloud = dynamic_cast<PointCloud*>(model);
    if (PointCloudNormals::estimate(cloud, k_neighbors)) {
        auto normals = cloud->get_vertex_property<vec3>("v:normal");
        auto drawable = cloud->renderer()->get_points_drawable("vertices");
        // Upload the vertex normals to the GPU.
        drawable->update_normal_buffer(normals.vector());
        viewer->update();
        return true;
    } else
        return false;
}

bool run_easy3d_ransac(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    auto points = cloud->get_vertex_property<vec3>("v:point");
    if (!normals) {
        bool estimate_normals = PointCloudNormals::estimate(cloud, k_neighbors);
        if (!estimate_normals) {
            LOG(WARNING) << "No normals found or estimated for point cloud";
            return false;
        }
    }

    // iterate over different parameters for RANSAC
    PrimitivesRansac ransac;

    ransac.add_primitive_type(PrimitivesRansac::CYLINDER);

    float normal_threshold = 0.8f;
    float overlook_probability = 0.001f;
    float bitmap_resolution = 0.05f;
    float dist_threshold = 0.01f;
    unsigned int min_support = 20;
    
    int num_cylinders = ransac.detect(cloud, min_support, dist_threshold, bitmap_resolution,
                                      normal_threshold, overlook_probability);

    // float best_bitmap_resolution = 0.0f;
    // unsigned int best_min_support = 0.0f;
    // float best_dist_threshold = 0.0f;
    // int num_cylinders = 0;

    // std::vector<float> bitmap_resolutions = {0.01f, 0.02f, 0.05f};
    // std::vector<float> min_support_ratios = {0.05, 0.1, 0.15, 0.2};
    // std::vector<float> dist_threshold_values = {0.002f, 0.005f, 0.01f};

    // for (float& min_support_ratio : min_support_ratios) {
    //     for (float& dist_threshold : dist_threshold_values) {
    //         for (float& bitmap_resolution : bitmap_resolutions) {
    //             unsigned int min_support = static_cast<unsigned int>(
    //                 std::round(min_support_ratio * cloud->n_vertices()));

    //             if (min_support < 3) min_support = 3;

    //             LOG(INFO) << "Testing min support: " << min_support
    //                       << ", dist threshold: " << dist_threshold;

    //             int detected_cylinders =
    //                 ransac.detect(cloud, min_support, dist_threshold, bitmap_resolution,
    //                               normal_threshold, overlook_probability);

    //             if (detected_cylinders > num_cylinders) {
    //                 num_cylinders = detected_cylinders;
    //                 best_min_support = min_support;
    //                 best_dist_threshold = dist_threshold;
    //                 best_bitmap_resolution = bitmap_resolution;
    //             }
    //         }
    //     }
    // }

    // LOG(INFO) << "Best min support: " << best_min_support
    //           << ", best dist threshold: " << best_dist_threshold
    //           << ", best bitmap resolution: " << best_bitmap_resolution;

    // num_cylinders = ransac.detect(cloud, best_min_support, best_dist_threshold,
    //                               best_bitmap_resolution, normal_threshold, overlook_probability);

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

        // clear previous viewer drawables
        for (auto& drawable : drawables) {
            viewer->delete_drawable(drawable);
        }
        drawables.clear();

        // draw bbox and cylinders
        for (int i = 0; i < cylinders.size(); i++) {
            auto cylinder = cylinders[i];
            LOG(INFO) << "Cylinder " << i << ": " << cylinder.position << " -- "
                      << cylinder.direction << " -- " << cylinder.radius << " -- "
                      << cylinder.vertices.size();
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
            const std::vector<vec3> bbox_points = {vec3(xmin, ymin, zmax), vec3(xmax, ymin, zmax),
                                                   vec3(xmin, ymax, zmax), vec3(xmax, ymax, zmax),
                                                   vec3(xmin, ymin, zmin), vec3(xmax, ymin, zmin),
                                                   vec3(xmin, ymax, zmin), vec3(xmax, ymax, zmin)};
            const std::vector<unsigned int> bbox_indices = {0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 4, 6,
                                                            1, 3, 5, 7, 0, 4, 2, 6, 1, 5, 3, 7};
            bbox_drawable->update_vertex_buffer(bbox_points);
            bbox_drawable->update_element_buffer(bbox_indices);
            bbox_drawable->set_uniform_coloring(vec4(0.0f, 0.0f, 1.0f, 1.0f));
            bbox_drawable->set_line_width(5.0f);
            viewer->add_drawable(bbox_drawable);
            drawables.push_back(bbox_drawable);

            auto cylinder_drawable = new LinesDrawable("cylinder");
            std::vector<vec3> cylinder_endpoints = {
                cylinder.position, cylinder.position + cylinder.direction * 100.0f};
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(cylinder.radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->add_drawable(cylinder_drawable);
            drawables.push_back(cylinder_drawable);
        }
    }
    return true;
}

bool run_cgal_ransac(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    auto points = cloud->get_vertex_property<vec3>("v:point");
    if (!normals) {
        bool estimate_normals = PointCloudNormals::estimate(cloud, k_neighbors);
        normals = cloud->get_vertex_property<vec3>("v:normal");
        if (!estimate_normals) {
            LOG(WARNING) << "No normals found or estimated for point cloud";
            return false;
        }
    }

    LOG(INFO) << "Constructing point_with_normal_vector";
    Pwn_vector pwn_vector;
    for (const auto& vertex : cloud->vertices()) {
        const easy3d::vec3 p = points[vertex];
        const easy3d::vec3 n = normals[vertex];
        pwn_vector.emplace_back(Kernel::Point_3(p[0], p[1], p[2]),
                                Kernel::Vector_3(n[0], n[1], n[2]));
    }

    LOG(INFO) << "Running CGAL RANSAC";
    Efficient_ransac ransac;

    ransac.set_input(pwn_vector);

    ransac.add_shape_factory<Cylinder>();

    Efficient_ransac::Parameters params;
    params.normal_threshold = 0.8;
    params.probability = 0.01;
    params.min_points = 20;
    params.epsilon = 0.7;
    params.cluster_epsilon = 1.0;

    ransac.detect(params);

    auto shapes = ransac.shapes();
    std::vector<Cylinder*> cylinders;
    for (auto& shape : shapes) {
        if (Cylinder* cylinder = dynamic_cast<Cylinder*>(shape.get())) {
            cylinders.push_back(cylinder);
        }
    }
    int num_cylinders = cylinders.size();

    LOG(INFO) << "Detected " << num_cylinders << " cylinders";

    if (num_cylinders > 0) {
        // clear previous viewer drawables
        for (auto& drawable : drawables) {
            viewer->delete_drawable(drawable);
        }
        drawables.clear();

        // draw bbox and cylinders
        for (auto& cylinder : cylinders) {
            const std::vector<std::size_t>& vertices = cylinder->indices_of_assigned_points();
            std::vector<vec3> cylinder_points;
            for (auto& vertex : vertices) {
                auto& point = pwn_vector.at(vertex).first;
                cylinder_points.push_back(vec3(point.x(), point.y(), point.z()));
            }

            auto bbox_drawable = new LinesDrawable("bbox");
            const Box3& box = geom::bounding_box<Box3, std::vector<vec3>>(cylinder_points);
            float xmin = box.min_coord(0);
            float xmax = box.max_coord(0);
            float ymin = box.min_coord(1);
            float ymax = box.max_coord(1);
            float zmin = box.min_coord(2);
            float zmax = box.max_coord(2);
            const std::vector<vec3> bbox_points = {vec3(xmin, ymin, zmax), vec3(xmax, ymin, zmax),
                                                   vec3(xmin, ymax, zmax), vec3(xmax, ymax, zmax),
                                                   vec3(xmin, ymin, zmin), vec3(xmax, ymin, zmin),
                                                   vec3(xmin, ymax, zmin), vec3(xmax, ymax, zmin)};
            const std::vector<unsigned int> bbox_indices = {0, 1, 2, 3, 4, 5, 6, 7, 0, 2, 4, 6,
                                                            1, 3, 5, 7, 0, 4, 2, 6, 1, 5, 3, 7};
            bbox_drawable->update_vertex_buffer(bbox_points);
            bbox_drawable->update_element_buffer(bbox_indices);
            bbox_drawable->set_uniform_coloring(vec4(0.0f, 0.0f, 1.0f, 1.0f));
            bbox_drawable->set_line_width(5.0f);
            viewer->add_drawable(bbox_drawable);
            drawables.push_back(bbox_drawable);

            auto cylinder_drawable = new LinesDrawable("cylinder");
            auto axis = cylinder->axis();
            auto random_point = axis.point();
            auto direction = axis.to_vector();
            auto end_point = random_point + direction;
            auto radius = cylinder->radius();
            std::vector<vec3> cylinder_endpoints = {
                vec3(random_point.x(), random_point.y(), random_point.z()),
                vec3(end_point.x(), end_point.y(), end_point.z())};
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->add_drawable(cylinder_drawable);
            drawables.push_back(cylinder_drawable);
        }
    }
    return true;
}
