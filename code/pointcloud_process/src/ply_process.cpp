#include <iostream>
#include <fstream>
#include <ply_process.h>
#include <tinyply.h>
#include <cstring>
#include <filesystem>


int main(int argc, char* argv[])
{
    if (argc != 5) {
        std::cerr << "Usage: " << argv[0] << " <input.ply> <semantic class name (e.g. sem_class)> <semantic class label value (int)> <instance class name (e.g. ins_class)>" << std::endl;
        return 1;
    }

    // read input arguments and check validity
    std::string input_file = argv[1];
    std::string sem_class_name = argv[2];
    std::string ins_class_name = argv[4];
    int sem_label_value;
    try {
        sem_label_value = std::stoi(argv[3]);
    } catch (const std::exception &e) {
        std::cerr << "Invalid label value: " << e.what() << std::endl;
        return 1;
    }
    std::ifstream ss(input_file, std::ios::binary);
    if (ss.fail()) {
        throw std::runtime_error("failed to open " + input_file);
    }

    // create a tinyply object and parse header info to it
    tinyply::PlyFile file;
    file.parse_header(ss);

    // read format and header information
    std::cout << "PLY Header Information:" << std::endl;
    std::cout << "=======================" << std::endl;

    std::cout << "Format: " << (file.is_binary_file() ? "Binary" : "ASCII") << std::endl;

    for (const auto &element : file.get_elements()) {
        std::cout << "Element: " << element.name << " (" << element.size << ")" << std::endl;

        for (const auto &property : element.properties) {
            std::cout << "Property: " << property.name << " (Type: "
             << tinyply::PropertyTable[property.propertyType].str << ")" << std::endl;
        }
    }
    std::cout << "=======================" << std::endl;

    std::cout << "Start requesting data..." << std::endl;
    // get vertices and corresponding semantic class
    std::shared_ptr<tinyply::PlyData> vertices, sem_class, instance_class;
    try { vertices = file.request_properties_from_element("testing", { "x", "y", "z" }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { sem_class = file.request_properties_from_element("testing", { sem_class_name }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    try { instance_class = file.request_properties_from_element("testing", { ins_class_name }); }
    catch (const std::exception & e) { std::cerr << "tinyply exception: " << e.what() << std::endl; }

    file.read(ss);

    std::cout << "Start retriving vertices..." << std::endl;
    std::vector<float3> vertex_data(vertices->count);
    std::memcpy(vertex_data.data(), vertices->buffer.get(), vertices->buffer.size_bytes());

    std::cout << "Start retriving semantic labels..." << std::endl;
    std::vector<label_t> sem_class_data(sem_class->count);
    std::memcpy(sem_class_data.data(), sem_class->buffer.get(), sem_class->buffer.size_bytes());

    std::cout << "Start retriving instance labels..." << std::endl;
    std::vector<label_t> instance_class_data(instance_class->count);
    std::memcpy(instance_class_data.data(), instance_class->buffer.get(), instance_class->buffer.size_bytes());

    std::cout << "Start processing data..." << std::endl;
    // get vertices with target semantic class label and their instance labels
    std::vector<float3> target_vertices;
    std::vector<label_t> ins_label_list;
    std::vector<label_t> unique_ins_label_list;

    for (size_t i = 0; i < sem_class_data.size(); ++i) {
        if (sem_class_data[i].label == sem_label_value) {
            target_vertices.push_back(vertex_data[i]);
            ins_label_list.push_back(instance_class_data[i]);
            unique_ins_label_list.push_back(instance_class_data[i]);
        }
    }

    // delete redundant labels
    std::sort(unique_ins_label_list.begin(), unique_ins_label_list.end());
    auto last = std::unique(unique_ins_label_list.begin(), unique_ins_label_list.end());
    unique_ins_label_list.erase(last, unique_ins_label_list.end());

    std::cout << "Start writing outputs..." << std::endl; 

    std::filesystem::path folderPath = "output";

    if (!std::filesystem::exists(folderPath)) {
        if (std::filesystem::create_directory(folderPath)) {
            std::cout << "Directory created successfully: " << folderPath << std::endl;
        } else {
            std::cerr << "Failed to create directory!" << std::endl;
        }
    } else {
        std::cout << "Directory already exists: " << folderPath << std::endl;
    }

    write_file(folderPath.string(), "xyz", target_vertices, ins_label_list, unique_ins_label_list);

    std::cout << "Finished." << std::endl;

    return 0;
}
