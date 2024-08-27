#include <iostream>
#include <fstream>
#include <tinyply.h>
#include <vector>

struct float3 { float x, y, z; };

struct label_t { 
    int label; 
    bool operator==(const label_t& other) const {
        return this->label == other.label;
    }

    bool operator<(const label_t& other) const {
        return this->label < other.label;
    }
};

void write_file(const std::string& folder, const std::string& format, std::vector<float3>& vertices, std::vector<label_t>& ins_label_list, const std::vector<label_t>& unique_ins_label_list)
{
    for (int i = 0; i < unique_ins_label_list.size(); i++) {
        std::string filePath = folder + "/output" + std::to_string(i) + "." + format;
        std::filebuf fb_binary;
        fb_binary.open(filePath, std::ios::out | std::ios::binary);
        std::ostream output_stream(&fb_binary);

        tinyply::PlyFile file;

        std::vector<float3> current_vertices;

        // use iterator for searching
        auto vertex_it = vertices.begin();
        auto label_it = ins_label_list.begin();

        while (vertex_it != vertices.end() && label_it != ins_label_list.end()) {
            if (label_it->label == unique_ins_label_list[i].label) {
                
                current_vertices.push_back(*vertex_it);

                // delete searched vertex and its label from vertices and ins_label_list
                vertex_it = vertices.erase(vertex_it);
                label_it = ins_label_list.erase(label_it);

            } else {
                vertex_it++;
                label_it++;
            }
        }

        if (format == "ply") {
            file.add_properties_to_element("vertex", { "x", "y", "z" }, tinyply::Type::FLOAT32, 
                                            current_vertices.size(), reinterpret_cast<const uint8_t*>(current_vertices.data()), tinyply::Type::INVALID, 0);

            file.write(output_stream, true);
        }
        else if (format == "xyz") {
            for (const auto& vertex : current_vertices) {
                output_stream << vertex.x << " " << vertex.y << " " << vertex.z << "\n";
            }
        }
    }
}