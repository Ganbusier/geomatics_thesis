#include <iostream>
#include <fstream>
#include <tinyply.h>

struct float3 { float x, y, z; };
struct label_t { int label; };

void write_ply(const std::string& filename, const std::vector<float3>& vertices, const std::vector<label_t>& label_values, const std::string& label_name)
{
    std::filebuf fb_binary;
    fb_binary.open(filename, std::ios::out | std::ios::binary);
    std::ostream output_stream(&fb_binary);

    tinyply::PlyFile file;

    file.add_properties_to_element("testing", { "x", "y", "z" }, tinyply::Type::FLOAT32, vertices.size(), reinterpret_cast<const uint8_t*>(vertices.data()), tinyply::Type::INVALID, 0);
    file.add_properties_to_element("testing", { label_name }, tinyply::Type::INT32, label_values.size(), reinterpret_cast<const uint8_t*>(label_values.data()), tinyply::Type::INVALID, 0);

    file.write(output_stream, true);
}

void write_xyz(const std::string& filename, const std::vector<float3>& vertices, const std::vector<label_t>& label_values)
{
    std::ofstream ofs(filename);
    if (!ofs.is_open()) {
        std::cerr << "Failed to open output file: " << filename << std::endl;
        return;
    }

    for (size_t i = 0; i < vertices.size(); ++i) {
        ofs << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << " " << label_values[i].label << "\n";
    }

    ofs.close();
}