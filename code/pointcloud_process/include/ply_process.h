#include <iostream>
#include <fstream>
#include <tinyply.h>

struct float3 { float x, y, z; };
struct label_t { char label[256]; };
void write_ply(const std::string& filename, const std::vector<float3>& vertices, const std::vector<label_t>& labels);