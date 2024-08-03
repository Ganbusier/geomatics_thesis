#include <iostream>
#include <fstream>
#include <ply_process.h>
#include <tinyply.h>


int main(int argc, char* argv[])
{
    if (argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> <output.ply> <label>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = argv[2];
    std::string label = argv[3];
    int target_label;
    try {
        target_label = std::stoi(argv[3]);
    } catch (const std::exception &e) {
        std::cerr << "Invalid label value: " << e.what() << std::endl;
        return 1;
    }
    std::ifstream ss(input_file, std::ios::binary);
    if (ss.fail()) {
        throw std::runtime_error("failed to open " + input_file);
    }

    tinyply::PlyFile file;
    file.parse_header(ss);

    std::shared_ptr<tinyply::PlyData> vertices, labels;
    try { vertices = file.request_properties_from_element("testing", { "x", "y", "z" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { labels = file.request_properties_from_element("testing", { "sem_class" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    file.read(ss);

    std::vector<float3> vertex_data(vertices->count);
    std::memcpy(vertex_data.data(), vertices->buffer.get(), vertices->buffer.size_bytes());

    std::vector<label_t> label_data(labels->count);
    std::memcpy(label_data.data(), labels->buffer.get(), labels->buffer.size_bytes());

    std::vector<float3> target_vertices;
    std::vector<label_t> target_labels;

    for (size_t i = 0; i < label_data.size(); ++i) {
        if (label_data[i].label == target_label) {
            target_vertices.push_back(vertex_data[i]);
            target_labels.push_back(label_data[i]);
        }
    }

    write_ply(output_file, target_vertices, target_labels);

    return 0;
}
