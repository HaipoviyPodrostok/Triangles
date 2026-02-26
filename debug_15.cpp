#include <fstream>
#include <iostream>
#include <vector>

#include "geometry/triangle.hpp"
#include "geometry/vector_3d.hpp"

using namespace geometry;

int main() {
  std::ifstream in("tests/end2end/tests/000015.txt");
  int n;
  in >> n;
  std::vector<Triangle> trs;
  for (int i = 0; i < n; i++) {
    double ax, ay, az, bx, by, bz, cx, cy, cz;
    in >> ax >> ay >> az >> bx >> by >> bz >> cx >> cy >> cz;
    trs.emplace_back(Vector3D{ax, ay, az}, Vector3D{bx, by, bz},
                     Vector3D{cx, cy, cz});
  }

  auto check_pair = [&](int i, int j) {
    bool intersect = trs[i].is_intersect(trs[j]);
    std::cout << "Triangle " << i << " x Triangle " << j << ": "
              << std::boolalpha << intersect << "\n";
  };

  check_pair(294, 295);  // Expected: ???
  return 0;
}
