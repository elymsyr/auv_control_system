#include "EnvironmentMap.h"
#include <iostream>
#include <cstdint>
#include <cuda_runtime.h>

void process_batches_with_slide(void* map, void** batches, int num_batches, float dx, float dy) {
    // 1. Slide the grid by dx/dy
    launch_slide_kernel(map, dx, dy);
    
    // 2. Update points from all batches
    for(int i = 0; i < num_batches; ++i) {
        launch_update_kernel(map, batches[i]);
    }
}

void fill_point_batch_with_random(void* batch, int grid_width, int grid_height) {
    PointBatch* h_batch = static_cast<PointBatch*>(batch);
    
    // Allocate host memory for random data
    int2* h_coords = new int2[h_batch->count];
    uint8_t* h_values = new uint8_t[h_batch->count];
    
    // Generate random points (host side)
    for(int i = 0; i < h_batch->count; ++i) {
        h_coords[i].x = rand() % grid_width * 20;    // Random x ∈ [0, width-1]
        h_coords[i].y = rand() % grid_height * 20;   // Random y ∈ [0, height-1]
        h_values[i] = rand() % 256;             // Random value ∈ [0,255]
        // std::cout << "Point " << i << ": (" 
        //           << h_coords[i].x << ", " 
        //           << h_coords[i].y << ") = " 
        //           << static_cast<int>(h_values[i]) << std::endl;
    }
    
    // Copy to device
    cudaMemcpy(h_batch->coords_dev, h_coords, 
               h_batch->count * sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(h_batch->values_dev, h_values,
               h_batch->count * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    // Cleanup host memory
    delete[] h_coords;
    delete[] h_values;
}

void initialize_test_pattern(void* map) {
    launch_slide_kernel(map, 0, 0);
    void* batch = create_point_batch(1);
    
    // Corrected: Remove 'struct' keyword
    PointBatch* h_batch = static_cast<PointBatch*>(batch);
    int2 center = make_int2(64, 64);
    uint8_t value = 255;
    
    cudaMemcpy(h_batch->coords_dev, &center, sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(h_batch->values_dev, &value, sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    launch_update_kernel(map, batch);
    destroy_point_batch(batch);
}

void update_single_point(void* map, PointBatch* h_batch, float world_x, float world_y, uint8_t value) {
    int2 coord = make_int2(static_cast<int>(world_x), static_cast<int>(world_y));
    uint8_t val = value;
    
    // Copy to device
    cudaMemcpy(h_batch->coords_dev, &coord, sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(h_batch->values_dev, &val, sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    // Launch kernel
    launch_update_kernel(map, h_batch);
}

int main() {
    const int W = 129, H = 129;
    
    // 1. Create and initialize map
    void* map = create_environment_map(W, H);
    
    void* batch_one = create_point_batch(1);
    PointBatch* h_batch = static_cast<PointBatch*>(batch_one);

    void* batch = create_point_batch(100);
    fill_point_batch_with_random(batch, W, H);

    update_single_point(map, h_batch, 322.0f, 123.0f, 255); // Update center point

    // 2. Create test pattern
    initialize_test_pattern(map);
    launch_update_kernel(map, batch);
    destroy_point_batch(batch);
    save_grid_to_file(map, "test_initial.bin");
    
    // 3. Apply shift
    const float dx = 56.0f, dy = 30.5f;
    launch_slide_kernel(map, dx, dy);
    save_grid_to_file(map, "test_shifted.bin");
    
    // 4. Cleanup
    destroy_point_batch(batch_one);
    destroy_environment_map(map);
    
    return 0;
}

// # 1. Compile CUDA code
// nvcc -arch=sm_75 -c EnvironmentMap.cu -o EnvironmentMap.o

// # 2. Compile main.cpp with nvcc (not g++)
// nvcc -arch=sm_75 -x cu -c main.cpp -o main.o

// # 3. Link
// nvcc -arch=sm_75 EnvironmentMap.o main.o -o main
// ./main