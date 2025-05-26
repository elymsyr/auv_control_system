// EnvironmentMapWrapper.cu
#include "EnvironmentMap.h"
#include <fstream>      // Add for file operations
#include <cstdint>      // Add for uint8_t
#include <vector_types.h>  // Add for int2

void save_grid_to_file(void* map, const char* filename) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    uint8_t* h_grid = new uint8_t[d_map->width * d_map->height];
    
    cudaMemcpy(h_grid, d_map->grid, 
              d_map->width * d_map->height * sizeof(uint8_t),
              cudaMemcpyDeviceToHost);
    
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<char*>(h_grid), 
             d_map->width * d_map->height);
    file.close();
    
    delete[] h_grid;
}

// Environment Map Management
void* create_environment_map(int w, int h) {
    EnvironmentMap* map;
    cudaMallocManaged(&map, sizeof(EnvironmentMap));
    map->initialize(w, h);
    return map;
}

void destroy_environment_map(void* map) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    d_map->cleanup();
    cudaFree(d_map);
}

// Point Batch Management
void* create_point_batch(int count) {
    PointBatch* batch;
    cudaMallocManaged(&batch, sizeof(PointBatch));
    cudaMalloc(&batch->coords_dev, count * sizeof(int2));
    cudaMalloc(&batch->values_dev, count * sizeof(uint8_t));
    batch->count = count;
    return batch;
}

void destroy_point_batch(void* batch) {
    PointBatch* d_batch = static_cast<PointBatch*>(batch);
    cudaFree(d_batch->coords_dev);
    cudaFree(d_batch->values_dev);
    cudaFree(d_batch);
}

void launch_slide_kernel(void* map, float dx, float dy) {
    auto start = std::chrono::high_resolution_clock::now();
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    dim3 threads(16, 16);
    dim3 blocks(
        (d_map->width + threads.x - 1) / threads.x,
        (d_map->height + threads.y - 1) / threads.y
    );
    iterateKernel<<<blocks, threads>>>(d_map, dx, dy);
    cudaDeviceSynchronize();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Slide kernel: " << duration.count() << "μs\n";
}

void launch_update_kernel(void* map, void* batch) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    PointBatch* d_batch = static_cast<PointBatch*>(batch);
    const int blockSize = 256;
    const int gridSize = (d_batch->count + blockSize - 1) / blockSize;
    pointUpdateKernel<<<gridSize, blockSize>>>(d_map, *d_batch);
    cudaDeviceSynchronize();
}