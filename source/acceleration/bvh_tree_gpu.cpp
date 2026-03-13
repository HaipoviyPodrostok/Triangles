#ifdef USE_OPENCL
#include <CL/opencl.h>

#include <fstream>
#include <iostream>

#include "acceleration/bvh_tree.hpp"

namespace {
struct CLContext {
  cl_context context = nullptr;
  cl_command_queue queue = nullptr;
  cl_program program = nullptr;
  cl_device_id device = nullptr;
  bool valid = false;
};

struct SplitInfo {
  std::vector<int32_t> splits;
  std::vector<int32_t> begins;
  std::vector<int32_t> ends;
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

std::optional<SplitInfo> run_find_splits(
    const CLContext& cl, const std::vector<uint32_t>& morton_codes) noexcept {
  cl_int err;
  const size_t num_objects = morton_codes.size();

  if (num_objects > static_cast<size_t>(std::numeric_limits<int32_t>::max())) {
    std::cerr << "BVH Build GPU Error: too many objects." << std::endl;
    return std::nullopt;
  }
  const size_t bytes = num_objects * sizeof(uint32_t);

  cl_mem d_morton_codes =
      clCreateBuffer(cl.context, CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, bytes,
                     const_cast<uint32_t*>(morton_codes.data()), &err);
  cl_mem d_splits =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  cl_mem d_begins =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);
  cl_mem d_ends =
      clCreateBuffer(cl.context, CL_MEM_WRITE_ONLY, bytes, NULL, &err);

  cl_kernel kernel = clCreateKernel(cl.program, "find_splits", &err);

  clSetKernelArg(kernel, 0, sizeof(cl_mem), &d_morton_codes);
  clSetKernelArg(kernel, 1, sizeof(cl_mem), &d_splits);
  clSetKernelArg(kernel, 2, sizeof(cl_mem), &d_begins);
  clSetKernelArg(kernel, 3, sizeof(cl_mem), &d_ends);

  const int32_t num_obj_int = static_cast<int32_t>(num_objects);
  clSetKernelArg(kernel, 4, sizeof(int32_t), &num_obj_int);

  size_t num_nodes = num_objects - 1;
  clEnqueueNDRangeKernel(cl.queue, kernel, 1, NULL, &num_nodes, NULL, 0, NULL,
                         NULL);
  ;

  const size_t bytes_out = num_nodes * sizeof(uint32_t);

  std::vector<int32_t> out_splits(num_nodes);
  std::vector<int32_t> begins(num_nodes);
  std::vector<int32_t> ends(num_nodes);

  clEnqueueReadBuffer(cl.queue, d_splits, CL_TRUE, 0, bytes, out_splits.data(),
                      0, NULL, NULL);
  clEnqueueReadBuffer(cl.queue, d_begins, CL_TRUE, 0, bytes, begins.data(), 0,
                      NULL, NULL);
  clEnqueueReadBuffer(cl.queue, d_ends, CL_TRUE, 0, bytes, ends.data(), 0, NULL,
                      NULL);

  clReleaseMemObject(d_splits);
  clReleaseMemObject(d_begins);
  clReleaseMemObject(d_ends);
  clReleaseMemObject(d_morton_codes);
  clReleaseKernel(kernel);

  return SplitInfo{out_splits, begins, ends};
}

}  // namespace

namespace acceleration {
template <typename PolT>
void acceleration::BVHTree<PolT>::build_gpu() noexcept {
  CLContext cl = init_opencl();

  // ... работаем ...
}
// Не забываем инстанцирование в конце
template class acceleration::BVHTree<geometry::Triangle>;
#endif
}  // acceleration