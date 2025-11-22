#include "acceleration/bvh_tree.hpp" // IWYU pragma: export
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <ostream>
#include <vector>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

using namespace geometry;

int main() {
    
    auto file_logger = spdlog::basic_logger_mt("file_logger", "log/app.log");
    spdlog::set_level(spdlog::level::debug);
    spdlog::set_default_logger(file_logger);
    spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
    spdlog::flush_on(spdlog::level::debug);
    spdlog::flush_on(spdlog::level::info);

    size_t tri_num = 0;

    std::cin >> tri_num;

    std::vector<Triangle> input;

    for (size_t i = 0; i < tri_num; ++i) {
        std::vector<Vector3D> points;
        for (size_t j = 0; j < 3; ++j) {
            float coords[3] = {};
            for (size_t k = 0; k < 3; ++k) {
                std::cin >> coords[k];
            }
            points.emplace_back(Vector3D{coords[0], coords[1], coords[2]});
        }
        input.emplace_back(Triangle{points[0], points[1], points[2]});
    }
    
    std::vector<uint8_t> is_intersected(tri_num, 0); 

    for (size_t i = 0; i < input.size(); ++i) {
        for (size_t j = i + 1; j < input.size(); ++j) {
            spdlog::debug("Intersection {} and {}:", i, j);
            if (input[i].is_intersect(input[j])) {
                spdlog::debug("yes");
                is_intersected[i] = 1;
                is_intersected[j] = 1;
            }
            else {
                spdlog::debug("no");
            }
        }
    }

    for (size_t i = 0; i < input.size(); ++i) {
        if (is_intersected[i] == 1) {
            std::cout << i  << std::endl;
        }
    }

    spdlog::info("Program finished");

    return 0;
}
