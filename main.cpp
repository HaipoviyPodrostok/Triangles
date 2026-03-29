#include <cstddef>
#include <iostream>
#include <vector>

#include "acceleration/acceleration.hpp"
#include "geometry/geometry.hpp"
#include "utils/logger.hpp"

#ifdef ENABLE_LOGGING
#include <spdlog/sinks/basic_file_sink.h>
#endif

using namespace geometry;

std::vector<Triangle> read_triangles(std::istream& in) {
  size_t tri_num = 0;
  if (!(in >> tri_num)) { return {}; }

  std::vector<Triangle> input;
  input.reserve(tri_num);

  for (size_t i = 0; i < tri_num; ++i) {
    std::vector<Vector3D> points;
    points.reserve(3);
    for (size_t j = 0; j < 3; ++j) {
      float coords[3] = {};
      for (size_t k = 0; k < 3; ++k) { in >> coords[k]; }
      points.emplace_back(Vector3D{coords[0], coords[1], coords[2]});
    }
    input.emplace_back(Triangle{points[0], points[1], points[2]});
  }

  return input;
}

int main() {
#ifdef ENABLE_LOGGING
  auto file_logger = spdlog::basic_logger_mt("file_logger", "log/app.log");
  spdlog::set_level(spdlog::level::debug);
  spdlog::set_default_logger(file_logger);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  spdlog::flush_on(spdlog::level::debug);
  spdlog::flush_on(spdlog::level::info);
#endif  // ENABLE_LOGGING

  LOG_INFO("Program started");

  std::vector<Triangle> input;

  input = read_triangles(std::cin);

  acceleration::BVHTree<Triangle> tree{input};

  std::vector<bool> output = tree.get_intersections();
  for (size_t i = 0; i < output.size(); ++i) {
    if (output[i]) { std::cout << i << std::endl; }
  }

  LOG_INFO("Program finished");

  return 0;
}
