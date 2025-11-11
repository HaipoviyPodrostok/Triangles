#include "acceleration/bvh_tree.hpp"
#include "geometry/geometry.hpp"
#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"
#include "math/math_utils.hpp"
#include <cstddef>
#include <fstream>
#include <ostream>
#include <vector>

using namespace geometry;
constexpr float eps = math::flt_tolerance;

int main() {
    size_t tri_num = 0;
 
    std::ifstream file("./assets/test1.txt");
    if (!file.is_open()) {
        std::cerr << "file open error" << std::endl;
        return 1;
    }
    
    file >> tri_num;

    std::vector<Triangle> input;

    for (size_t i = 0; i < tri_num; ++i) {
        std::vector<Vector3D> points;
        for (size_t j = 0; j < 3; ++j) {
            float coords[3] = {};
            for (size_t k = 0; k < 3; ++k) {
                file >> coords[k];
            }
            points.emplace_back(Vector3D{coords[0], coords[1], coords[2]});
        }
        input.emplace_back(Triangle{points[0], points[1], points[2]});
    }

    size_t intersect_tri_cnt = 0;
    
    for (size_t i = 0; i < input.size(); ++i) {
        for (size_t j = i + 1; j < input.size(); ++j) {
            if (input[i].intersection(input[j])) {
                ++intersect_tri_cnt;
            }
        }
    }

    std::cout << intersect_tri_cnt << std::endl;
    return 0;
}
