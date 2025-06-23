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

Path EnvironmentMap::findPath() {
    // Convert world goal to grid coordinates
    int goal_x = (ref_.x - world_position_.x) / r_m_ + width_ / 2;
    int goal_y = (ref_.y - world_position_.y) / r_m_ + height_ / 2;
    goal_x = std::max(0, std::min(goal_x, width_ - 1));
    goal_y = std::max(0, std::min(goal_y, height_ - 1));

    // Step 1: Reset grid
    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);
              
    resetGridKernel<<<grid, block>>>(node_grid_, grid_, width_, height_, goal_x, goal_y);
    CUDA_CALL(cudaDeviceSynchronize());

    // Step 2: Wavefront propagation
    int* d_updated;
    int h_updated = 1;
    CUDA_CALL(cudaMalloc(&d_updated, sizeof(int)));
    
    for (int iter = 0; iter < max_iter_ && h_updated; iter++) {
        h_updated = 0;
        CUDA_CALL(cudaMemcpy(d_updated, &h_updated, sizeof(int), cudaMemcpyHostToDevice));
        
        wavefrontKernel<<<grid, block>>>(node_grid_, width_, height_, d_updated);
        CUDA_CALL(cudaDeviceSynchronize());
        CUDA_CALL(cudaMemcpy(&h_updated, d_updated, sizeof(int), cudaMemcpyDeviceToHost));
    }

    // Step 3: Reconstruct path
    int2* d_path;
    int* d_path_length;
    CUDA_CALL(cudaMalloc(&d_path, width_ * height_ * sizeof(int2)));
    CUDA_CALL(cudaMalloc(&d_path_length, sizeof(int)));
    CUDA_CALL(cudaMemset(d_path_length, 0, sizeof(int)));
    
    reconstructPathKernel<<<1, 1>>>(node_grid_, d_path, d_path_length, 
                                   start_x, start_y, goal_x, goal_y, width_);
    CUDA_CALL(cudaDeviceSynchronize());
    
    int path_length;
    CUDA_CALL(cudaMemcpy(&path_length, d_path_length, sizeof(int), cudaMemcpyDeviceToHost));
    
    int2* h_path = new int2[path_length];
    if (path_length > 0) {
        CUDA_CALL(cudaMemcpy(h_path, d_path, path_length * sizeof(int2), cudaMemcpyDeviceToHost));
    }

    // Cleanup
    CUDA_CALL(cudaFree(d_updated));
    CUDA_CALL(cudaFree(d_path));
    CUDA_CALL(cudaFree(d_path_length));
    
    return {h_path, path_length};
}