#include "EnvironmentMap.h"
#include <iostream>
#include <cstdint>
#include <cuda_runtime.h>

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

int main() {
    const int W = 129, H = 129;
    
    // 1. Create and initialize map
    void* map = create_environment_map(W, H);
    
    // 2. Create test pattern
    initialize_test_pattern(map);
    save_grid_to_file(map, "test_initial.bin");
    
    // 3. Apply shift
    const int shiftX = 5, shiftY = 3;
    launch_slide_kernel(map, shiftX, shiftY);
    save_grid_to_file(map, "test_shifted.bin");
    
    // 4. Cleanup
    destroy_environment_map(map);
    
    return 0;
}