#include "acceleration/bvh_tree.hpp" // IWYU pragma: export
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <ostream>
#include <vector>

using namespace geometry;
constexpr float eps = math::flt_tolerance;

int main() {
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
            if (input[i].intersection(input[j])) {
                is_intersected[i] = 1;
                is_intersected[j] = 1;
            }
        }
    }

    for (size_t i = 0; i < input.size(); ++i) {
        if (is_intersected[i] == 1) {
            std::cout << i  << std::endl;
        }
    }

    return 0;
}
