#include <algorithm>
#include <cassert>
#include <cstddef>
#include <fstream>

#include "acceleration/bvh_tree.hpp"

#ifdef USE_OPENCL
#include <CL/opencl.h>
#endif  // USE_OPENCL

#include <spdlog/spdlog.h>

namespace acceleration {

#ifdef USE_OPENCL

namespace {

[[nodiscard]] uint32_t expand_bits(uint32_t v) {
  v &= 0x000003ff;

  v = (v | (v << 16)) & 0x030000ff;
  v = (v | (v << 8)) & 0x0300f00f;
  v = (v | (v << 4)) & 0x030c30c3;
  v = (v | (v << 2)) & 0x09249249;
  return v;
}

}  // namespace

[[nodiscard]] std::vector<uint32_t> detail::get_morton_code(
    const std::vector<geometry::Vector3D>& centroids,
    const acceleration::AABB&              box) noexcept {
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

namespace {

struct CLContext {
  cl_context       context = nullptr;
  cl_command_queue queue   = nullptr;
  cl_program       program = nullptr;
  cl_device_id     device  = nullptr;

  bool valid = false;
};

struct CLMem {
  cl_mem handle = nullptr;

  CLMem(cl_mem h) : handle(h) {}
  ~CLMem() {
    if (handle) clReleaseMemObject(handle);
  }

  CLMem(const CLMem&)            = delete;
  CLMem& operator=(const CLMem&) = delete;
};

CLContext init_opencl() {
  CLContext      cl;
  cl_int         err;
  cl_platform_id platform;

  clGetPlatformIDs(1, &platform, NULL);
  err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &cl.device, NULL);
  if (err != CL_SUCCESS) { throw std::runtime_error("OpenCL: GPU not found"); }
  cl.context = clCreateContext(NULL, 1, &cl.device, NULL, NULL, &err);
  cl.queue = clCreateCommandQueueWithProperties(cl.context, cl.device, 0, &err);

  std::ifstream file(acceleration::opencl_file);
  if (!file.is_open()) {
    throw std::runtime_error(
        "OpenCL: couldn't open the kernel file on the path " +
        acceleration::opencl_file);
  }

  std::string source_code((std::istreambuf_iterator<char>(file)),
                          std::istreambuf_iterator<char>());

  const char*  source_str  = source_code.c_str();
  const size_t source_size = source_code.length();

  cl.program =
      clCreateProgramWithSource(cl.context, 1, &source_str, &source_size, &err);
  err = clBuildProgram(cl.program, 1, &cl.device, NULL, NULL, NULL);

  if (err == CL_BUILD_PROGRAM_FAILURE) {
    throw std::runtime_error("OpenCL: Kernel compilation error");
  }

  cl.valid = true;
  return cl;
}

void cleanup_opencl(CLContext& cl) {
  if (cl.program) { clReleaseProgram(cl.program); }
  if (cl.queue) { clReleaseCommandQueue(cl.queue); }
  if (cl.context) { clReleaseContext(cl.context); }
}

detail::SplitInfo run_find_splits(const CLContext&             cl,
                                  const std::vector<uint32_t>& morton_codes) {
  cl_int       err;
  const size_t num_objects = morton_codes.size();

  if (num_objects > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
    throw std::length_error("Too many objects");
  }
  const size_t bytes = num_objects * sizeof(uint32_t);

  CLMem morton_codes_buf =
      clCreateBuffer(cl.context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, bytes,
                     const_cast<uint32_t*>(morton_codes.data()), &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failure to build a tree on the GPU");
  }

  CLMem splits_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failure to build a tree on the GPU");
  }

  CLMem starts_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failure to build a tree on the GPU");
  }

  CLMem n_objs_buf =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  if (err != CL_SUCCESS) {
    throw std::runtime_error("Failure to build a tree on the GPU");
  }

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

  err = clEnqueueReadBuffer(cl.queue, splits_buf.handle, CL_TRUE, 0, bytes_out,
                            out_splits.data(), 0, NULL, NULL);
  if (err != CL_SUCCESS)
    throw std::runtime_error("OpenCL: Error copying splits back");

  err = clEnqueueReadBuffer(cl.queue, starts_buf.handle, CL_TRUE, 0, bytes_out,
                            starts.data(), 0, NULL, NULL);
  if (err != CL_SUCCESS)
    throw std::runtime_error("OpenCL: Error copying splits back");

  err = clEnqueueReadBuffer(cl.queue, n_objs_buf.handle, CL_TRUE, 0, bytes_out,
                            n_objs.data(), 0, NULL, NULL);
  if (err != CL_SUCCESS)
    throw std::runtime_error("OpenCL: Error copying splits back");

  clReleaseKernel(kernel);

  return detail::SplitInfo{out_splits, starts, n_objs};
}

}  // namespace

detail::SplitInfo detail::get_split_info(
    const std::vector<uint32_t>& morton_codes) {
  CLContext cl = init_opencl();

  const detail::SplitInfo split_info = run_find_splits(cl, morton_codes);
  cleanup_opencl(cl);

  spdlog::info("get_split_info successful)");
  return split_info;
}

void detail::fill_node_idx(std::vector<BVHNode>&    nodes,
                           const detail::SplitInfo& split_info) {
  const size_t n_internals = split_info.splits.size();
  const size_t n_leafs     = n_internals + 1;

  nodes.resize(n_internals + n_leafs);

  spdlog::info("resize succes");

  for (size_t i = 0; i < n_internals; ++i) {
    int32_t left_idx  = -1;
    int32_t right_idx = -1;

    const int32_t split = split_info.splits[i];
    const int32_t start = split_info.starts[i];
    const int32_t end   = start + split_info.n_objs[i] - 1;

    if (split == start) {
      left_idx = n_internals + split;
    } else {
      left_idx = split;
    }

    if (split + 1 == end) {
      right_idx = n_internals + (split + 1);
    } else {
      right_idx = split + 1;
    }

    nodes[i].left_idx  = left_idx;
    nodes[i].right_idx = right_idx;
    nodes[i].start     = 0;
    nodes[i].n_objs    = 0;
  }

  for (size_t i = 0; i < n_leafs; ++i) {
    nodes[n_internals + i].start     = i;
    nodes[n_internals + i].n_objs    = 1;
    nodes[n_internals + i].left_idx  = -1;
    nodes[n_internals + i].right_idx = -1;
  }

  spdlog::info("fill_node succes");
}

#endif  // USE_OPENCL
}  // namespace acceleration