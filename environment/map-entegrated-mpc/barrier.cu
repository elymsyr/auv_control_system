#include "EnvironmentMap.h"
#include <cmath>

__device__ float barrier_function(const EnvironmentMap* map, float x, float y) {
    // Convert world coordinates to grid coordinates
    float grid_x = (x - map->x_r_) / map->x_r_cm_ + map->width / 2.0f;
    float grid_y = (y - map->y_r_) / map->y_r_cm_ + map->height / 2.0f;
    
    int ix = static_cast<int>(roundf(grid_x));
    int iy = static_cast<int>(roundf(grid_y));
    
    if (ix >= 0 && ix < map->width && iy >= 0 && iy < map->height) {
        uint8_t val = map->grid[iy * map->width + ix];
        // Convert PointID to distance: obstacles are negative
        if (val == static_cast<uint8_t>(3)) {  // 3 = OBSTACLE
            return -1.0f;  // Unsafe region
        }
        return 1.0f;  // Safe region
    }
    return 0.0f;  // Unknown region
}

__global__ void barrier_kernel(const EnvironmentMap* map, const float* positions, float* outputs, int n) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx < n) {
        float x = positions[2*idx];
        float y = positions[2*idx+1];
        outputs[idx] = barrier_function(map, x, y);
    }
}

extern "C" void barrier_func(const float* positions, float* outputs, int n, void* map_ptr) {
    EnvironmentMap* map = static_cast<EnvironmentMap*>(map_ptr);
    const int blockSize = 256;
    const int gridSize = (n + blockSize - 1) / blockSize;
    barrier_kernel<<<gridSize, blockSize>>>(map, positions, outputs, n);
    cudaDeviceSynchronize();
}