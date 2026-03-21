#include "acceleration/bvh_tree.hpp"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <fstream>
#include <iostream>

#ifdef USE_OPENCL
#include <CL/opencl.h>
#endif  // USE_OPENCL

#include "acceleration/acceleration.hpp"

namespace acceleration {

BVHNode::BVHNode(const AABB& box_, const size_t first_, const size_t n_objs_)
    : box(box_), start(first_), n_objs(n_objs_) {
  assert(box_.is_valid());
}

bool BVHNode::is_valid() const noexcept {
  if (!box.is_valid()) { return false; }

  if (is_leaf()) {
    return left_idx == -1 && right_idx == -1 && n_objs > 0 &&
           n_objs <= max_leaf_capacity;
  } else {
    return n_objs == 0 && left_idx > -1 && right_idx > -1 &&
           left_idx != right_idx;
  }
}

void BVHNode::init_leaf(const AABB& box_, const size_t start_,
                        const size_t n_objs_) {
  box = box_;
  start = start_;
  n_objs = n_objs_;
  left_idx = -1;
  right_idx = -1;
}

void BVHNode::init_internal(const AABB& box_, const int left_idx_,
                            const int right_idx_) {
  box = box_;
  start = 0;
  n_objs = 0;
  left_idx = left_idx_;
  right_idx = right_idx_;
}

[[nodiscard]] uint32_t expand_bits(uint32_t v) noexcept {
  v &= 0x000003ff;

  v = (v | (v << 16)) & 0x030000ff;
  v = (v | (v << 8)) & 0x0300f00f;
  v = (v | (v << 4)) & 0x030c30c3;
  v = (v | (v << 2)) & 0x09249249;
  return v;
}

[[nodiscard]] std::vector<uint32_t> detail::get_morton_code(
    const std::vector<geometry::Vector3D>& centroids,
    const acceleration::AABB& box) {
  const double x_gap = std::max(box.max.x - box.min.x, 1e-9);
  const double y_gap = std::max(box.max.y - box.min.y, 1e-9);
  const double z_gap = std::max(box.max.z - box.min.z, 1e-9);

  std::vector<uint32_t> morton_codes;

  for (size_t i = 0; i < centroids.size(); ++i) {
    const double norm_x =
        ((centroids[i].x - box.min.x) / x_gap) * grid_resolution;
    const double norm_y =
        ((centroids[i].y - box.min.y) / y_gap) * grid_resolution;
    const double norm_z =
        ((centroids[i].z - box.min.z) / z_gap) * grid_resolution;

    uint32_t ix = static_cast<uint32_t>(
        std::clamp(norm_x, 0.0, static_cast<double>(grid_resolution - 1)));
    uint32_t iy = static_cast<uint32_t>(
        std::clamp(norm_y, 0.0, static_cast<double>(grid_resolution - 1)));
    uint32_t iz = static_cast<uint32_t>(
        std::clamp(norm_z, 0.0, static_cast<double>(grid_resolution - 1)));

    const uint32_t morton_code =
        (expand_bits(ix) << 2) | (expand_bits(iy) << 1) | (expand_bits(iz));
    morton_codes.push_back(morton_code);
  }

  return morton_codes;
}

// [[nodiscard]] size_t find_common_pref(const size_t a, const size_t b) {
//   const uint32_t xor_val = a ^ b;

//   if (xor_val == 0) { return 32; }

// #if defined(__GNUC__) || defined(__clang__)
//   return __builtin_clz(xor_val);
// #elif defined(_MSC_VER)
//   unsigned long index;
//   _BitScanReverse(&index, xor_val);
//   return 31 - static_cast<size_t>(index);
// #else
//   int n = 0;
//   if ((xor_val & 0xFFFF0000) == 0) {
//     n += 16;
//     xor_val <<= 16;
//   }
//   if ((xor_val & 0xFF000000) == 0) {
//     n += 8;
//     xor_val <<= 8;
//   }
//   if ((xor_val & 0xF0000000) == 0) {
//     n += 4;
//     xor_val <<= 4;
//   }
//   if ((xor_val & 0xC0000000) == 0) {
//     n += 2;
//     xor_val <<= 2;
//   }
//   if ((xor_val & 0x80000000) == 0) { n += 1; }
//   return n;
// #endif
// }

// [[nodiscard]] size_t find_split(const std::vector<uint32_t>& morton_codes,
//                                 const size_t first, const size_t last) {
//   const uint32_t first_code = morton_codes[first];
//   const uint32_t last_code = morton_codes[last];

//   if (first_code == last_code) { return (first + last) >> 1; }

// #ifdef USE_OPENCL
//   const size_t common_prefix = clz(first_code ^ last_code);
// #endif

// #ifndef USE_OPENCL
//   const size_t common_prefix = std::countl_zero(first_code ^ last_code);
// #endif

//   size_t split = first;
//   size_t step = last - first;

//   do {
//     step = (step + 1) >> 1;
//     const size_t new_split = split + step;

//     if (new_split < last) {
//       const uint32_t split_code = morton_codes[new_split];
// #ifndef USE_OPENCL
//       const size_t split_prefix =
//     }
//   }
// }
// }  // namespace

struct CLContext {
  cl_context context = nullptr;
  cl_command_queue queue = nullptr;
  cl_program program = nullptr;
  cl_device_id device = nullptr;
  bool valid = false;
};

struct CLMem {
  cl_mem handle = nullptr;

  CLMem(cl_mem h) : handle(h) {}
  ~CLMem() {
    if (handle) clReleaseMemObject(handle);
  }

  CLMem(const CLMem&) = delete;
  CLMem& operator=(const CLMem&) = delete;
};

CLContext init_opencl() noexcept {
  CLContext cl;
  cl_int err;
  cl_platform_id platform;
  clGetPlatformIDs(1, &platform, NULL);
  clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &cl.device, NULL);

  cl.context = clCreateContext(NULL, 1, &cl.device, NULL, NULL, &err);
  cl.queue = clCreateCommandQueueWithProperties(cl.context, cl.device, 0, &err);
  std::ifstream file(acceleration::opencl_file);
  std::string source_code((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());

  const char* source_str = source_code.c_str();
  const size_t source_size = source_code.length();
  cl.program =
      clCreateProgramWithSource(cl.context, 1, &source_str, &source_size, &err);
  err = clBuildProgram(cl.program, 1, &cl.device, NULL, NULL, NULL);

  if (err == CL_BUILD_PROGRAM_FAILURE) {
    size_t log_size;
    clGetProgramBuildInfo(cl.program, cl.device, CL_PROGRAM_BUILD_LOG, 0, NULL,
                          &log_size);
    std::vector<char> build_log(log_size);
    clGetProgramBuildInfo(cl.program, cl.device, CL_PROGRAM_BUILD_LOG, log_size,
                          build_log.data(), NULL);
    std::cerr << "Build log:\n" << build_log.data() << std::endl;
    return cl;
  }
  cl.valid = true;
  return cl;
}

void cleanup_opencl(CLContext& cl) noexcept {
  if (cl.program) clReleaseProgram(cl.program);
  if (cl.queue) clReleaseCommandQueue(cl.queue);
  if (cl.context) clReleaseContext(cl.context);
}

std::optional<detail::SplitInfo> run_find_splits(
    const CLContext& cl, const std::vector<uint32_t>& morton_codes) noexcept {
  cl_int err;
  const size_t num_objects = morton_codes.size();

  if (num_objects > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
    std::cerr << "BVH Build GPU Error: too many objects." << std::endl;
    return std::nullopt;
  }
  const size_t bytes = num_objects * sizeof(uint32_t);

  CLMem morton_codes_buf =
      clCreateBuffer(cl.context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, bytes,
                     const_cast<uint32_t*>(morton_codes.data()), &err);
  if (err != CL_SUCCESS) { return std::nullopt; }

  CLMem splits_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) { return std::nullopt; }

  CLMem starts_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) { return std::nullopt; }

  CLMem n_objs_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) { return std::nullopt; }

  const cl_kernel kernel = clCreateKernel(cl.program, "find_splits", &err);

  clSetKernelArg(kernel, 0, sizeof(cl_mem), &morton_codes_buf.handle);
  clSetKernelArg(kernel, 1, sizeof(cl_mem), &splits_buf.handle);
  clSetKernelArg(kernel, 2, sizeof(cl_mem), &starts_buf.handle);
  clSetKernelArg(kernel, 3, sizeof(cl_mem), &n_objs_buf.handle);

  const int32_t num_obj_int = static_cast<int32_t>(num_objects);
  clSetKernelArg(kernel, 4, sizeof(int32_t), &num_obj_int);

  size_t num_nodes = num_objects - 1;
  clEnqueueNDRangeKernel(cl.queue, kernel, 1, NULL, &num_nodes, NULL, 0, NULL,
                         NULL);
  ;

  const size_t bytes_out = num_nodes * sizeof(uint32_t);

  std::vector<int32_t> out_splits(num_nodes);
  std::vector<int32_t> starts(num_nodes);
  std::vector<int32_t> n_objs(num_nodes);

  clEnqueueReadBuffer(cl.queue, splits_buf.handle, CL_TRUE, 0, bytes_out,
                      out_splits.data(), 0, NULL, NULL);
  clEnqueueReadBuffer(cl.queue, starts_buf.handle, CL_TRUE, 0, bytes_out,
                      starts.data(), 0, NULL, NULL);
  clEnqueueReadBuffer(cl.queue, n_objs_buf.handle, CL_TRUE, 0, bytes_out,
                      n_objs.data(), 0, NULL, NULL);

  clReleaseKernel(kernel);

  return detail::SplitInfo{out_splits, starts, n_objs};
}

std::optional<detail::SplitInfo> detail::get_split_info(
    const std::vector<uint32_t>& morton_codes) {
  CLContext cl = init_opencl();
  const std::optional<detail::SplitInfo> split_info =
      run_find_splits(cl, morton_codes);
  cleanup_opencl(cl);
  return split_info;
}

void fill_nodes_idx(std::vector<BVHNode>& nodes,
                    const detail::SplitInfo& split_info) {
  const size_t n_nodes = split_info.splits.size();
  const size_t n_leafs = n_nodes + 1;

  nodes.resize(n_nodes + n_leafs);

  for (size_t i = 0; i < n_nodes - 1; ++i) {
    int32_t left_idx = -1;
    int32_t right_idx = -1;

    const int32_t split = split_info.splits[i];
    const int32_t start = split_info.starts[i];
    const int32_t end = start + split_info.n_objs[i] - 1;

    if (split == start) {
      left_idx = n_nodes + split;
    } else {
      left_idx = split;
    }

    if (split + 1 == end) {
      right_idx = n_nodes + (split + 1);
    } else {
      right_idx = split + 1;
    }

    nodes[i].left_idx = left_idx;
    nodes[i].right_idx = right_idx;
    nodes[i].start = start;
    nodes[i].n_objs = split_info.n_objs[i];
  }
}

void compute_boxes(std::vector<BVHNode>& nodes, const int inernal)

//
// BVHTree::BVHTree(std::vector<geometry::Triangle>& input) :
// triangles(input)
// {
//   if (input.size() < 2) {
//     throw std::logic_error("There must be at least 2 triangles");
//   }

//   build_node(0, input.size(), 0);
// }

// bool BVHTree::is_valid() const {
//   return root >= 0;
// }

// AABB BVHTree::calculate_box(const size_t start, const size_t n_objs) {
//   AABB box{triangles[start]};
//   for (size_t i = start + 1; i < start + n_objs; ++i) {
//     box.expand(triangles[i]);
//   }
//   return box;
// }

// int BVHTree::partition_by_median(const size_t start, const size_t n_objs) {
//   const AABB& box = calculate_box(start, n_objs);

//   const double spread_x = std::fabs(box.max.x - box.min.x);
//   const double spread_y = std::fabs(box.max.y - box.min.y);
//   const double spread_z = std::fabs(box.max.z - box.min.z);

//   math::Axis wildest_axis =
//       (spread_x >= spread_y && spread_x >= spread_z)
//           ? math::Axis::X
//           : (spread_y >= spread_z ? math::Axis::Y : math::Axis::Z);

//   const size_t mid_idx = start + n_objs / 2;

//   sort_triangles_by_centers(start, n_objs, wildest_axis, mid_idx);

//   return mid_idx;
// }

// void BVHTree::sort_triangles_by_centers(const size_t start, const size_t
// n_objs,
//                                         const math::Axis wildest_axis,
//                                         const size_t mid_idx) {
//   auto begin = triangles.begin() + start;
//   auto mid = triangles.begin() + mid_idx;
//   auto end = triangles.begin() + start + n_objs;

//   switch (wildest_axis) {
//     case math::Axis::X: {
//       auto centre_cmp = [](const geometry::Triangle& a,
//                            const geometry::Triangle& b) {
//         return a.get_centre().x < b.get_centre().x;
//       };
//       std::nth_element(begin, mid, end, centre_cmp);
//       break;
//     }
//     case math::Axis::Y: {
//       auto centre_cmp = [](const geometry::Triangle& a,
//                            const geometry::Triangle& b) {
//         return a.get_centre().y < b.get_centre().y;
//       };
//       std::nth_element(begin, mid, end, centre_cmp);
//       break;
//     }
//     default: {
//       auto centre_cmp = [](const geometry::Triangle& a,
//                            const geometry::Triangle& b) {
//         return a.get_centre().z < b.get_centre().z;
//       };
//       std::nth_element(begin, mid, end, centre_cmp);
//       break;
//     }
//   }
// }

// int BVHTree::build_node(const size_t start, const size_t n_objs,
//                         const size_t depth) {
//   int node_idx = nodes.size();
//   nodes.emplace_back();

//   if (depth > max_depth_reached) { max_depth_reached = depth; }

//   if (n_objs <= max_leaf_capacity || depth >= tree_max_depth) {
//     nodes[node_idx].init_leaf(calculate_box(start, n_objs), start, n_objs);
//     return node_idx;
//   }

//   const int mid_idx = partition_by_median(start, n_objs);

//   const size_t left_n_objs = mid_idx - start;
//   const size_t right_n_objs = n_objs - left_n_objs;

//   const int left_idx = build_node(start, left_n_objs, depth + 1);
//   const int right_idx = build_node(mid_idx, right_n_objs, depth + 1);

//   nodes[node_idx].init_internal(
//       merge(nodes[left_idx].box, nodes[right_idx].box), left_idx,
//       right_idx);

//   return node_idx;
// }

// #ifdef ENABLE_BVH_DEBUG
// void BVHTree::dump() {
//   if (root == -1) { throw std::invalid_argument("tree is invalid"); }

//   std::ofstream dump_file = make_dump_file();
// }

// void BVHTree::dump_recursion(const int node_idx) {
//   assert(node_idx >= 0);

//   const BVHNode& node = nodes[node_idx];

//   if (node.left_idx >= 0) { dump_recursion(node.left_idx); }
//   if (node.right_idx >= 0) { dump_recursion(node.right_idx); }

//   node.print_node();
// }

// std::ofstream BVHTree::make_dump_file() {
//   namespace fs = std::filesystem;
//   fs::path dump_folder_path = default_dump_folder;
//   if (!fs::exists(dump_folder_path)) {
//     fs::create_directories(dump_folder_path);
//   }

//   std::fstream fin{counter_file_name, std::ios::in | std::ios::out};
//   int last_launch_num = -1;

//   if (fin.is_open()) {
//     std::string line;
//     std::getline(fin, line);
//     std::istringstream num_string(line.substr(17));
//     num_string >> last_launch_num;
//     last_launch_num++;
//     fin << "Last launch_num = " << last_launch_num;
//   } else {
//     std::ofstream fout(counter_file_name);
//     fout << "Last launch_num = 1";
//     last_launch_num = 1;
//   }

//   dump_call_cnt++;

//   std::string dump_file_name = std::string("bvh_tree_dump_") +
//                                std::to_string(last_launch_num) + "_" +
//                                std::to_string(dump_call_cnt) + ".dot";

//   std::ofstream dump_file{dump_file_name};
//   return dump_file;
// }
// #endif  // ENABLE_BVH_DEBUG

// geometry::Vector3D node_box_min = nodes[idx].box.min;
// geometry::Vector3D node_box_max = nodes[idx].box.max;

// geometry::Vector3D left_min  = node_box_min;
// geometry::Vector3D right_max = node_box_max;

// geometry::Vector3D left_max;
// geometry::Vector3D right_min;

// switch(wildest_axis) {
//     case math::Axis::X: {
//         const float split_x = (nodes[idx].box.max.x + nodes[idx].box.min.x)
//         * 0.5f; left_max  = geometry::Vector3D{ split_x, node_box_max.y,
//         node_box_max.z }; right_min = geometry::Vector3D{ split_x,
//         node_box_min.y, node_box_min.z }; break;
//     }
//     case math::Axis::Y: {
//         const float split_y = (nodes[idx].box.max.y + nodes[idx].box.min.y)
//         * 0.5f; left_max  = geometry::Vector3D{ node_box_max.x, split_y,
//         node_box_max.z }; right_min = geometry::Vector3D{ node_box_min.x,
//         split_y, node_box_min.z }; break;
//     }
//     default: {
//         const float split_z = (nodes[idx].box.max.z + nodes[idx].box.min.z)
//         * 0.5f; left_max  = geometry::Vector3D{ node_box_max.x,
//         node_box_max.y, split_z }; right_min = geometry::Vector3D{
//         node_box_min.x, node_box_min.y, split_z }; break;
//     }
// }

// AABB left{left_min, left_max};
// node.left = nodes.size();
// nodes.emplace_back(left);

// AABB right{right_min, right_max};
// node.right = nodes.size();
// nodes.emplace_back(right);

// std::sort(start, end,
//           [wildest_axis](const geometry::Triangle& a, const
//           geometry::Triangle& b) {
//             switch (wildest_axis) {
//                 case math::Axis::X:
//                     return a.get_centre().x < b.get_centre().x;
//                 case math::Axis::Y:
//                     return a.get_centre().y < b.get_centre().y;
//                 default:
//                     return a.get_centre().z < b.get_centre().z;
//             }
//           });

// AABB left_box{triangles[start]};
// for (size_t i = start + 1; i < mid_idx; ++i) {
//     left_box.expand(triangles[i]);
// }

// AABB right_box{triangles[mid_idx]};
// for (size_t i = mid_idx + 1; i < start + n_objs; ++i) {
//     right_box.expand(triangles[i]);
// }

// const size_t left_n_objs = mid_idx - start; //mid_idx -> right box
// const size_t right_n_objs = n_objs - left_n_objs;

// if (left_n_objs == 0 || right_n_objs == 0) {
//     is_leaf = true;
//     return;
// }

// const int left_child_idx = nodes.size();
// nodes.emplace_back(BVHNode{left_box, node.start, left_n_objs});

// const int right_child_idx = nodes.size();
// nodes.emplace_back(BVHNode{right_box, mid_idx, right_n_objs});

// node.left_idx = left_child_idx;
// node.right_idx = right_child_idx;

// partition_by_median(left_child_idx);
// partition_by_median(right_child_idx);
}  // namespace acceleration