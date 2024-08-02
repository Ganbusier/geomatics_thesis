#include <iostream>
#include <fstream>
#include <ply_process.h>
#include <tinyply.h>


int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input.ply>" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::ifstream ss(input_file, std::ios::binary);
    if (ss.fail()) {
        throw std::runtime_error("failed to open " + input_file);
    }

    tinyply::PlyFile file;
    file.parse_header(ss);

    std::shared_ptr<tinyply::PlyData> vertices, labels;
    try { vertices = file.request_properties_from_element("vertex", { "x", "y", "z" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { labels = file.request_properties_from_element("vertex", { "label" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    file.read(ss);

    std::vector<float3> vertex_data(vertices->count);
    std::memcpy(vertex_data.data(), vertices->buffer.get(), vertices->buffer.size_bytes());

    std::vector<label_t> label_data(labels->count);
    std::memcpy(label_data.data(), labels->buffer.get(), labels->buffer.size_bytes());

    std::vector<float3> poles_vertices;
    std::vector<label_t> poles_labels;

    for (size_t i = 0; i < label_data.size(); ++i) {
        if (std::string(label_data[i].label) == "poles") {
            poles_vertices.push_back(vertex_data[i]);
            poles_labels.push_back(label_data[i]);
        }
    }

    write_ply("output_poles.ply", poles_vertices, poles_labels);

    return 0;
}

void write_ply(const std::string& filename, const std::vector<float3>& vertices, const std::vector<label_t>& labels)
{
    std::filebuf fb_binary;
    fb_binary.open(filename, std::ios::out | std::ios::binary);
    std::ostream output_stream(&fb_binary);

    tinyply::PlyFile file;

    file.add_properties_to_element("vertex", { "x", "y", "z" }, tinyply::Type::FLOAT32, vertices.size(), reinterpret_cast<const uint8_t*>(vertices.data()), tinyply::Type::INVALID, 0);
    file.add_properties_to_element("vertex", { "label" }, tinyply::Type::UINT8, labels.size(), reinterpret_cast<const uint8_t*>(labels.data()), tinyply::Type::INVALID, 0);

    file.write(output_stream, true);
}
