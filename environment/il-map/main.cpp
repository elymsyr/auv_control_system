// main.cpp
#include "EnvironmentMap.h"
#include <iostream>

void initialize_test_pattern(void* map) {
    // Set center pixel to max value
    launch_slide_kernel(map, 0, 0); // Ensure no shift
    void* batch = create_point_batch(1);
    
    // Create single-point batch
    PointBatch* h_batch = static_cast<PointBatch*>(batch);
    int2 center = {64, 64}; // For 129x129 grid
    uint8_t value = 255;
    
    cudaMemcpy(h_batch->coords_dev, &center, sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(h_batch->values_dev, &value, sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    launch_update_kernel(map, batch);
    destroy_point_batch(batch);
}

int main() {
    void* map = create_environment_map(129, 129);
    
    // Test 1: Single point
    initialize_test_pattern(map);
    save_grid_to_file(map, "test1_initial.bin");
    
    // Test 2: Shift right
    launch_slide_kernel(map, 1, 0);
    save_grid_to_file(map, "test1_shifted.bin");
    
    destroy_environment_map(map);
    return 0;
}