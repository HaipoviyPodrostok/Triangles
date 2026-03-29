#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "acceleration/acceleration.hpp"
#include "geometry/geometry.hpp"

using namespace geometry;

std::vector<Triangle> read_triangles_from_file(const std::string& filepath) {
  std::ifstream file(filepath);
  if (!file.is_open()) {
    throw std::runtime_error("Cannot open file: " + filepath);
  }

  size_t tri_num = 0;
  if (!(file >> tri_num)) { return {}; }

  std::vector<Triangle> input;
  input.reserve(tri_num);

  for (size_t i = 0; i < tri_num; ++i) {
    std::vector<Vector3D> points;
    points.reserve(3);
    for (size_t j = 0; j < 3; ++j) {
      float coords[3] = {};
      for (size_t k = 0; k < 3; ++k) { file >> coords[k]; }
      points.emplace_back(Vector3D{coords[0], coords[1], coords[2]});
    }
    input.emplace_back(Triangle{points[0], points[1], points[2]});
  }

  return input;
}

int main(int argc, char* argv[]) {
  auto file_logger = spdlog::basic_logger_mt("file_logger", "log/app.log");
  spdlog::set_level(spdlog::level::debug);
  spdlog::set_default_logger(file_logger);
  spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");
  spdlog::flush_on(spdlog::level::debug);
  spdlog::flush_on(spdlog::level::info);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <filepath>\n";
    return 1;
  }

  std::vector<Triangle> input;
  try {
    input = read_triangles_from_file(argv[1]);
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return 1;
  }

  acceleration::BVHTree<Triangle> tree{input};

  tree.dump_to_dot("tree.dot");

  if (tree.validate_tree()) {
    std::cout << "valid" << std::endl;
  } else {
    std::cout << "non valid" << std::endl;
  }

  std::cout << "finished" << std::endl;

  std::vector<bool> output = tree.get_intersections();
  for (size_t i = 0; i < output.size(); ++i) {
    if (output[i]) { std::cout << i << std::endl; }
  }

  spdlog::info("Program finished");

  return 0;
}
