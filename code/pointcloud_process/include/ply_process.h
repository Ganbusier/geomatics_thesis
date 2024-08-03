#include <iostream>
#include <fstream>
#include <tinyply.h>

struct float3 { float x, y, z; };
struct label_t { int label; };

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
