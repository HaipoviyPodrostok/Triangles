// source/acceleration/kernels/lbvh.cl

// Вспомогательная функция для нахождения общего префикса между двумя кодами Мортона
// Обратите внимание: типы uint (чистый OpenCL C)
inline int get_common_prefix(uint a, uint b) {
    uint xor_val = a ^ b;
    if (xor_val == 0) return 32;
    // clz - встроенная аппаратная функция видеокарты
    return clz(xor_val); 
}

// Это ядро (kernel), которое будет запущено тысячами копий (потоков)
// Оно должно иметь классический префикс __kernel, 
// а все массивы из оперативной памяти должны быть помечены как __global
__kernel void find_splits(__global const uint* sorted_morton_codes,
                          __global int* out_splits,
                          const int num_objects) 
{
    // Получаем уникальный номер текущего потока видеокарты (от 0 до num_objects - 2)
    // Этот номер соответствует номеру внутреннего узла дерева, который мы строим
    int idx = get_global_id(0);
    
    // Защита от выхода за пределы массива
    if (idx >= num_objects - 1) return;

    // ... здесь будет логика алгоритма Карраса
    // Например:
    uint first_code = sorted_morton_codes[0]; // (упрощенно)
    uint last_code = sorted_morton_codes[num_objects - 1]; // (упрощенно)
    
    int common_prefix = get_common_prefix(first_code, last_code);
    
    // ... и цикл do { ... } while (step > 1);
    
    // out_splits[idx] = split;
}
