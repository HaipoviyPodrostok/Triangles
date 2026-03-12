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
  
  out_splits[idx] = split;
}
