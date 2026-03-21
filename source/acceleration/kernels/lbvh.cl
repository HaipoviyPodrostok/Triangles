inline int get_common_prefix(__global const uint* codes, const int i, 
                             const int j, const int num_objects) {
    
  if (i < 0 || i >= num_objects || j < 0 || j >= num_objects) {
      return -1;
  }
  
  const uint a = codes[i];
  const uint b = codes[j];
  
  if (a == b) {
      return 32 + clz((uint)(i ^ j));
  }
  
  return clz(a ^ b);
}

__kernel void find_splits(__global const uint* sorted_morton_codes,
                          __global int* out_splits,
                          __global int* starts,
                          __global int* n_objs,
                          const int num_objects) {
  const int idx = get_global_id(0);
  
  if (idx >= num_objects - 1) return;

  const int prefix_left  = get_common_prefix(sorted_morton_codes, idx, idx - 1, num_objects);
  const int prefix_right = get_common_prefix(sorted_morton_codes, idx, idx + 1, num_objects);
  
  const int d = (prefix_right > prefix_left) ? 1 : -1;
  
  const int prefix_min = min(prefix_left, prefix_right);

  int l_max = 2;
  while (get_common_prefix(sorted_morton_codes, idx, idx + l_max * d, num_objects) 
         > prefix_min) {
      l_max *= 2;
  }
  
  int l = 0;
  for (int t = l_max / 2; t >= 1; t /= 2) {
      if (get_common_prefix(sorted_morton_codes, idx, idx + (l + t) * d, num_objects) > prefix_min) {
          l += t;
      }
  }
  const int edge = idx + l * d;
  
  const int prefix_node = get_common_prefix(sorted_morton_codes, idx, edge, num_objects);
  
  int s = 0;
  int step = l;
  
  do {
      step = (step + 1) >> 1;
      if (get_common_prefix(sorted_morton_codes, idx, idx + (s + step) * d, num_objects) > prefix_node) {
          s += step;
      }
  } while (step > 1);
  
  const int split = idx + s * d + min(d, 0);
  const int start = min(edge, idx);
  const int end   = max(edge, idx);
  
  out_splits[idx] = split;
  starts[idx]     = start;
  n_objs[idx]     = end - start + 1;
}

// __kernel void compute_morton_codes(
//   __global const float* centroids_x,
//   __global const float* centroids_y,
//   __global const float* centroids_z,
//   __global uint* out_morton_codes,
//   const float min_x, const float min_y, const float min_z,
//   const float max_x, const float max_y, const float max_z,
//   const int num_objects)
//   {
//   const int idx = get_global_id(0);
//   if (idx >= num_objects) return;

//   float nx = (centroids_x[idx] - min_x) / (max_x - min_x);
//   float ny = (centroids_y[idx] - min_y) / (max_y - min_y);
//   float nz = (centroids_z[idx] - min_z) / (max_z - min_z);

//   uint ix = min((uint)(nx * 1024.0f), 1023u);
//   uint iy = min((uint)(ny * 1024.0f), 1023u);
//   uint iz = min((uint)(nz * 1024.0f), 1023u);

//   ix = (ix | (ix << 16)) & 0x030000FF;
//   ix = (ix | (ix <<  8)) & 0x0300F00F;
//   ix = (ix | (ix <<  4)) & 0x030C30C3;
//   ix = (ix | (ix <<  2)) & 0x09249249;

//   iy = (iy | (iy << 16)) & 0x030000FF;
//   iy = (iy | (iy <<  8)) & 0x0300F00F;
//   iy = (iy | (iy <<  4)) & 0x030C30C3;
//   iy = (iy | (iy <<  2)) & 0x09249249;

//   iz = (iz | (iz << 16)) & 0x030000FF;
//   iz = (iz | (iz <<  8)) & 0x0300F00F;
//   iz = (iz | (iz <<  4)) & 0x030C30C3;
//   iz = (iz | (iz <<  2)) & 0x09249249;

//   out_morton_codes[idx] = (ix << 2) | (iy << 1) | iz;
//   }

