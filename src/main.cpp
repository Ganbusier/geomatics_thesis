#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/IO/read_points.h>
#include <CGAL/Point_set_3.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Region_growing/Point_set.h>
#include <CGAL/Shape_detection/Region_growing/Region_growing.h>
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

// Typedefs for CGAL RANSAC
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

// Typedefs for CGAL Region growing
using Kernel_rg = CGAL::Simple_cartesian<double>;
using Point_3 = Kernel_rg::Point_3;
using Vector_3 = Kernel_rg::Vector_3;

using Point_set = CGAL::Point_set_3<Point_3>;
using Point_map_rg = typename Point_set::Point_map;
using Normal_map_rg = typename Point_set::Vector_map;

using Neighbor_query = CGAL::Shape_detection::Point_set::K_neighbor_query_for_point_set<Point_set>;
using Region_type =
    CGAL::Shape_detection::Point_set::Least_squares_cylinder_fit_region_for_point_set<Point_set>;
using Region_growing = CGAL::Shape_detection::Region_growing<Neighbor_query, Region_type>;

using namespace easy3d;

bool run_easy3d_ransac(Viewer* viewer, Model* model);
bool run_cgal_ransac(Viewer* viewer, Model* model);
bool estimate_normals(Viewer* viewer, Model* model);
bool offset_xyz(Viewer* viewer, Model* model);
bool run_cgal_region_growing(Viewer* viewer, Model* model);

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
    offset_xyz(&viewer, model);
    // set up rendering parameters
    auto drawable = model->renderer()->get_points_drawable("vertices");
    drawable->set_uniform_coloring(vec4(0.6f, 0.6f, 1.0f, 1.0f));
    drawable->set_impostor_type(PointsDrawable::PLAIN);
    drawable->set_point_size(3.0f);

    // usage
    viewer.set_usage(
        "'Ctrl + n': estimate normals\n"
        "'Ctrl + e': run CGAL RANSAC\n"
        "'Shift + e': run Easy3D RANSAC");
    viewer.bind(run_easy3d_ransac, model, Viewer::KEY_E, Viewer::MODIF_SHIFT);
    viewer.bind(run_cgal_ransac, model, Viewer::KEY_E, Viewer::MODIF_CTRL);
    viewer.bind(run_cgal_region_growing, model, Viewer::KEY_R, Viewer::MODIF_CTRL);
    viewer.bind(estimate_normals, model, Viewer::KEY_N, Viewer::MODIF_CTRL);

    // fit screen
    viewer.fit_screen();

    return viewer.run();
}

bool show_normals(Viewer* viewer, PointCloud* cloud) {
    if (!viewer || !cloud) return false;
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    auto points = cloud->get_vertex_property<vec3>("v:point");
    auto drawable = cloud->renderer()->get_points_drawable("vertices");

    // Upload the vertex normals to the GPU.
    drawable->update_normal_buffer(normals.vector());
    drawable->set_visible(true);
    viewer->update();

    // clear previous viewer drawables
    for (auto& drawable : drawables) {
        viewer->delete_drawable(drawable);
    }
    drawables.clear();

    for (auto vertex : cloud->vertices()) {
        auto normal = normals[vertex];
        auto point = points[vertex];
        auto line_drawable = new LinesDrawable("normal");
        std::vector<vec3> line_points = {point, point + normal * 10.0f};
        std::vector<unsigned int> cylinder_indices = {0, 1};
        line_drawable->update_vertex_buffer(line_points);
        line_drawable->update_element_buffer(cylinder_indices);
        line_drawable->set_impostor_type(LinesDrawable::PLAIN);
        line_drawable->set_line_width(1.0f);
        line_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
        viewer->add_drawable(line_drawable);
        drawables.push_back(line_drawable);
    }
    return true;
}

bool estimate_normals(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;
    auto cloud = dynamic_cast<PointCloud*>(model);
    auto normals = cloud->get_vertex_property<vec3>("v:normal");
    if (!normals) {
        if (PointCloudNormals::estimate(cloud, k_neighbors)) {
            show_normals(viewer, cloud);
            return true;
        } else {
            return false;
        }
    } else {
        show_normals(viewer, cloud);
        return true;
    }

    return true;
}

bool offset_xyz(Viewer* viewer, Model* model) {
    if (!viewer || !model) return false;

    auto cloud = dynamic_cast<PointCloud*>(model);
    auto points = cloud->get_vertex_property<vec3>("v:point");
    float min_x = INFINITY;
    float min_y = INFINITY;
    float min_z = INFINITY;
    for (auto vertex : cloud->vertices()) {
        const vec3& point = points[vertex];
        if (point.x < min_x) min_x = point.x;
        if (point.y < min_y) min_y = point.y;
        if (point.z < min_z) min_z = point.z;
    }
    min_x -= 1.0f;
    min_y -= 1.0f;
    min_z -= 1.0f;
    LOG(INFO) << "Offsetting xyz by " << min_x << ", " << min_y << ", " << min_z;
    LOG(INFO) << "Original point 0: " << points[PointCloud::Vertex(0)];
    for (auto vertex : cloud->vertices()) {
        points[vertex].x -= min_x;
        points[vertex].y -= min_y;
        points[vertex].z -= min_z;
    }
    LOG(INFO) << "Offset point 0:" << points[PointCloud::Vertex(0)];
    cloud->add_vertex_property<vec3>("v:offset_vector", vec3(min_x, min_y, min_z));
    return true;
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

    if (num_cylinders > 0) {
        LOG(INFO) << "Detected " << num_cylinders << " cylinders";
        auto cylinders = ransac.get_cylinders();
        auto segments = cloud->vertex_property<int>("v:primitive_index");
        const std::string color_name = "v:color-segments";
        auto coloring = cloud->vertex_property<vec3>(color_name, vec3(0.0f));
        Renderer::color_from_segmentation(cloud, segments, coloring);

        auto drawable = cloud->renderer()->get_points_drawable("vertices");
        drawable->set_visible(true);
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
            std::vector<vec3> cylinder_endpoints = {cylinder.position,
                                                    cylinder.position + cylinder.direction * 10.0f};
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(cylinder.radius * 2.0f);
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

    ransac.set_input(pwn_vector);  // the pwn_vector will be reordered after RANSAC

    ransac.add_shape_factory<Cylinder>();

    Efficient_ransac::Parameters params;
    params.normal_threshold = 0.9;
    params.probability = 0.01;
    params.min_points = 2;
    params.epsilon = 0.05;
    params.cluster_epsilon = 0.5;

    ransac.detect(params);

    auto shapes = ransac.shapes();
    std::vector<Cylinder*> cylinders;
    for (auto& shape : shapes) {
        if (Cylinder* cylinder = dynamic_cast<Cylinder*>(shape.get())) {
            cylinders.push_back(cylinder);
        }
    }
    int num_cylinders = cylinders.size();

    LOG(INFO) << "Detected " << num_cylinders << " cylinders, "
              << ransac.number_of_unassigned_points() << " unassigned points.";

    if (num_cylinders > 0) {
        // clear previous viewer drawables
        for (auto& drawable : drawables) {
            viewer->delete_drawable(drawable);
        }
        drawables.clear();

        // hide default point cloud
        auto default_drawable = cloud->renderer()->get_points_drawable("vertices");
        default_drawable->set_visible(false);
        default_drawable->update();
        viewer->update();

        // build new point cloud
        PointCloud* new_cloud = new PointCloud;
        auto new_points = new_cloud->get_vertex_property<vec3>("v:point");
        auto new_normals = new_cloud->add_vertex_property<vec3>("v:normal");
        // initialize segments property to -1 which means unknown primitive type
        auto segments = new_cloud->add_vertex_property<int>("v:primitive_index", -1);
        for (size_t i = 0; i < pwn_vector.size(); i++) {
            auto pwn = pwn_vector[i];
            auto& point = pwn.first;
            auto& normal = pwn.second;
            new_cloud->add_vertex(vec3(point.x(), point.y(), point.z()));
            new_normals[PointCloud::Vertex(i)] = vec3(normal.x(), normal.y(), normal.z());
        }

        for (size_t i = 0; i < cylinders.size(); i++) {
            auto cylinder = cylinders[i];
            const std::vector<std::size_t>& indices = cylinder->indices_of_assigned_points();
            for (auto& index : indices) {
                PointCloud::Vertex v(index);
                segments[v] = i;
            }
        }

        // draw new point cloud
        const std::string color_name = "v:color-segments";
        auto coloring = new_cloud->vertex_property<vec3>(color_name, vec3(0.0f));
        Renderer::color_from_segmentation(new_cloud, segments, coloring);

        auto drawable = new PointsDrawable("vertices");
        drawable->set_property_coloring(State::VERTEX, color_name);
        drawable->set_impostor_type(PointsDrawable::PLAIN);
        drawable->set_point_size(3.0f);

        drawable->update_vertex_buffer(new_points.vector());
        drawable->update_normal_buffer(new_normals.vector());
        drawable->update_color_buffer(coloring.vector());

        viewer->add_drawable(drawable);
        drawables.push_back(drawable);

        // draw bbox and cylinders
        for (int i = 0; i < cylinders.size(); i++) {
            auto cylinder = cylinders[i];
            LOG(INFO) << "Cylinder " << i << ": " << cylinder->info();
            const std::vector<std::size_t>& indices = cylinder->indices_of_assigned_points();
            std::vector<vec3> cylinder_points;
            for (auto& index : indices) {
                cylinder_points.push_back(new_points[PointCloud::Vertex(index)]);
            }

            auto bbox_drawable = new LinesDrawable("bbox");
            const Box3& box = geom::bounding_box<Box3, std::vector<vec3>>(cylinder_points);
            LOG(INFO) << "Box " << i << " center: " << box.center();
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
            auto direction = axis.to_vector();
            auto center = axis.point(0);
            auto start_point = center - direction * box.radius();
            auto end_point = center + direction * box.radius();
            auto radius = cylinder->radius();
            std::vector<vec3> cylinder_endpoints = {
                vec3(start_point.x(), start_point.y(), start_point.z()),
                vec3(end_point.x(), end_point.y(), end_point.z())};
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(2.0 * radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->add_drawable(cylinder_drawable);
            drawables.push_back(cylinder_drawable);
        }
    }
    return true;
}

bool run_cgal_region_growing(Viewer* viewer, Model* model) {
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

    LOG(INFO) << "Constructing point set for region growing";
    Point_set point_set;
    for (const auto& vertex : cloud->vertices()) {
        const easy3d::vec3 p = points[vertex];
        const easy3d::vec3 n = normals[vertex];
        point_set.insert(Point_3(p[0], p[1], p[2]), Vector_3(n[0], n[1], n[2]));
    }
    LOG(INFO) << "Point set size: " << point_set.size();

    LOG(INFO) << "Running CGAL region growing";

    // set up region growing parameters
    const std::size_t k = 20;
    const FT max_distance = FT(1.0);
    const FT max_angle = FT(25);
    const FT min_radius = FT(0.1);
    const FT max_radius = FT(5.0);
    const std::size_t min_region_size = 5;

    // create instances of the classes Neighbor_query and Region_type
    Neighbor_query neighbor_query = CGAL::Shape_detection::Point_set::make_k_neighbor_query(
        point_set, CGAL::parameters::k_neighbors(k));

    Region_type region_type =
        CGAL::Shape_detection::Point_set::make_least_squares_cylinder_fit_region(
            point_set, CGAL::parameters::maximum_distance(max_distance)
                           .maximum_angle(max_angle)
                           .minimum_radius(min_radius)
                           .maximum_radius(max_radius)
                           .minimum_region_size(min_region_size));

    // create an instance of the class Region_growing
    Region_growing region_growing(point_set, neighbor_query, region_type);

    // run the region growing algorithm
    CGAL::Random random;
    std::size_t num_cylinders = 0;
    std::size_t num_unassigned_points = point_set.size();
    std::vector<typename Region_growing::Primitive_and_region> regions;
    region_growing.detect(std::back_inserter(regions));

    LOG(INFO) << "Detected " << regions.size() << " cylinders.";

    if (!regions.empty()) {
        for (auto& region : regions) {
            const auto& indices = region.second;
            num_unassigned_points -= indices.size();
        }
    }
    LOG(INFO) << "Number of unassigned points: " << num_unassigned_points;

    if (!regions.empty()) {
        for (auto& drawable : drawables) {
            viewer->delete_drawable(drawable);
        }
        drawables.clear();

        // hide default point cloud
        auto default_drawable = cloud->renderer()->get_points_drawable("vertices");
        default_drawable->set_visible(false);
        default_drawable->update();
        viewer->update();

        // build new point cloud
        PointCloud* new_cloud = new PointCloud;
        auto new_points = new_cloud->get_vertex_property<vec3>("v:point");
        auto new_normals = new_cloud->add_vertex_property<vec3>("v:normal");
        // initialize segments property to -1 which means unknown primitive type
        auto segments = new_cloud->add_vertex_property<int>("v:primitive_index", -1);
        for (size_t i = 0; i < point_set.size(); ++i) {
            const auto& point = point_set.point(i);
            const auto& normal = point_set.normal(i);
            new_cloud->add_vertex(vec3(point.x(), point.y(), point.z()));
            new_normals[PointCloud::Vertex(i)] = vec3(normal.x(), normal.y(), normal.z());
        }

        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& primitive_and_region = regions[i];
            const auto& indices = primitive_and_region.second;
            for (auto& index : indices) {
                PointCloud::Vertex v(index);
                segments[v] = i;
            }
        }

        // draw new point cloud
        const std::string color_name = "v:color-segments";
        auto coloring = new_cloud->vertex_property<vec3>(color_name, vec3(0.0f));
        Renderer::color_from_segmentation(new_cloud, segments, coloring);

        auto drawable = new PointsDrawable("vertices");
        drawable->set_property_coloring(State::VERTEX, color_name);
        drawable->set_impostor_type(PointsDrawable::PLAIN);
        drawable->set_point_size(3.0f);

        drawable->update_vertex_buffer(new_points.vector());
        drawable->update_normal_buffer(new_normals.vector());
        drawable->update_color_buffer(coloring.vector());

        viewer->add_drawable(drawable);
        drawables.push_back(drawable);

        for (size_t i = 0; i < regions.size(); ++i) {
            const auto& primitive_and_region = regions[i];
            const auto& cylinder = primitive_and_region.first;
            LOG(INFO) << "Cylinder " << i << " center: " << cylinder.axis.point(0)
                      << " radius: " << cylinder.radius
                      << " direction: " << cylinder.axis.to_vector();

            const auto& indices = primitive_and_region.second;
            std::vector<vec3> cylinder_points;
            for (auto& index : indices) {
                cylinder_points.push_back(new_points[PointCloud::Vertex(index)]);
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
            auto axis = cylinder.axis;
            auto center_point = axis.point(0);
            auto direction = axis.to_vector();
            auto start_point = center_point + direction * box.radius();
            auto end_point = center_point - direction * box.radius();
            auto radius = cylinder.radius;
            std::vector<vec3> cylinder_endpoints = {
                vec3(start_point.x(), start_point.y(), start_point.z()),
                vec3(end_point.x(), end_point.y(), end_point.z())};
            std::vector<unsigned int> cylinder_indices = {0, 1};
            cylinder_drawable->update_vertex_buffer(cylinder_endpoints);
            cylinder_drawable->update_element_buffer(cylinder_indices);
            cylinder_drawable->set_impostor_type(LinesDrawable::CYLINDER);
            cylinder_drawable->set_line_width(2.0 * radius);
            cylinder_drawable->set_uniform_coloring(vec4(1.0f, 0.0f, 0.0f, 1.0f));
            viewer->add_drawable(cylinder_drawable);
            drawables.push_back(cylinder_drawable);
        }
    }

    return true;
}
