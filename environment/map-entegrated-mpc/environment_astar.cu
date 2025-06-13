#include <cmath>
#include <vector>
#include <cuda_runtime.h>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "environment.h"
#include <fstream>

void EnvironmentMap::initializeGrid() {
    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);

    initKernel<<<grid, block>>>(node_grid_, width_, height_);
    CHECK_CUDA(cudaDeviceSynchronize());
}

// Path EnvironmentMap::findPath() {
//     if (goal_x_ < 0 || goal_y_ < 0) return Path{nullptr, 0};

//     // Run A* kernel
//     dim3 block(16, 16);
//     dim3 grid((width_ + block.x - 1) / block.x, 
//               (height_ + block.y - 1) / block.y);
//     size_t shared_mem = block.x * block.y * sizeof(bool);
    
//     aStarKernel<<<grid, block, shared_mem>>>(d_grid_, obstacles, width_, height_,
//         start_x_, start_y_, goal_x_, goal_y_);
//     CHECK_CUDA(cudaDeviceSynchronize());

//     // Compute path length
//     int* d_length;
//     CHECK_CUDA(cudaMalloc(&d_length, sizeof(int)));
//     computePathLength<<<1,1>>>(d_grid_, width_, height_, 
//                               start_x_, start_y_, goal_x_, goal_y_, d_length);
//     CHECK_CUDA(cudaDeviceSynchronize());

//     int length;
//     CHECK_CUDA(cudaMemcpy(&length, d_length, sizeof(int), cudaMemcpyDeviceToHost));
//     CHECK_CUDA(cudaFree(d_length));

//     Path path_result = {nullptr, 0};
//     if (length <= 0) return path_result;

//     // Allocate and reconstruct path
//     CHECK_CUDA(cudaMalloc(&path_result.points, length * sizeof(int2)));
//     reconstructPath<<<1,1>>>(d_grid_, width_, height_, 
//                             start_x_, start_y_, goal_x_, goal_y_, 
//                             path_result.points, length);
//     CHECK_CUDA(cudaDeviceSynchronize());
    
//     path_result.length = length;
//     return path_result;
// }

// void EnvironmentMap::save(const char* filename) const {
//     // Allocate host memory for grid
//     Node* h_grid = new Node[width_ * height_];
    
//     // Copy device grid to host
//     CHECK_CUDA(cudaMemcpy(h_grid, d_grid_, 
//                          width_ * height_ * sizeof(Node),
//                          cudaMemcpyDeviceToHost));
    
//     // Create buffer for f values
//     float* f_values = new float[width_ * height_];
    
//     for (int idy = 0; idy < height_; idy++) {
//         for (int idx = 0; idx < width_; idx++) {
//             int index = idy * width_ + idx;
//             f_values[index] = h_grid[index].f;
//         }
//     }

//     std::ofstream file(filename, std::ios::binary);
//     file.write(reinterpret_cast<char*>(h_grid), width_ * height_);
//     file.close();
    
//     delete[] h_grid;
//     delete[] f_values;
// }