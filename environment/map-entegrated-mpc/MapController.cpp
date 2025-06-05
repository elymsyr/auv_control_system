#include "MapController.h"
#include "EnvironmentMap.h"
#include <iostream>
#include <cstdlib>
#include <cstring>

MapController::MapController(int width, int height) 
    : width_(width), height_(height) {
    map_ = static_cast<EnvironmentMap*>(create_environment_map(width, height));
    single_batch_ = static_cast<PointBatch*>(create_point_batch(1));
}

MapController::~MapController() {
    destroy_point_batch(single_batch_);
    destroy_environment_map(map_);
}

// Neural network access functions
uint8_t* MapController::get_grid_device_ptr() const {
    return map_->grid;
}

void MapController::copy_grid_to_host(uint8_t* host_buffer) const {
    size_t grid_size = width_ * height_ * sizeof(uint8_t);
    cudaMemcpy(host_buffer, map_->grid, grid_size, cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
}

// Map operations
void MapController::process_batches_with_slide(void** batches, int num_batches, float dx, float dy) {
    launch_slide_kernel(map_, dx, dy);
    for(int i = 0; i < num_batches; ++i) {
        launch_update_kernel(map_, batches[i]);
    }
}

void MapController::fill_point_batch_with_random(void* batch, int grid_width, int grid_height) {
    PointBatch* h_batch = static_cast<PointBatch*>(batch);
    int2* h_coords = new int2[h_batch->count];
    uint8_t* h_values = new uint8_t[h_batch->count];
    
    for(int i = 0; i < h_batch->count; ++i) {
        h_coords[i].x = rand() % (grid_width * 20);
        h_coords[i].y = rand() % (grid_height * 20);
        h_values[i] = rand() % 256;
    }
    
    cudaMemcpy(h_batch->coords_dev, h_coords, 
               h_batch->count * sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(h_batch->values_dev, h_values,
               h_batch->count * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    delete[] h_coords;
    delete[] h_values;
}

void MapController::initialize_test_pattern() {
    slide(0, 0);  // Reset position
    update_single_point(64.0f, 64.0f, 255);  // Center point
}

void MapController::update_single_point(float world_x, float world_y, uint8_t value) {
    int2 coord = make_int2(static_cast<int>(world_x), static_cast<int>(world_y));
    uint8_t val = value;
    
    cudaMemcpy(single_batch_->coords_dev, &coord, sizeof(int2), cudaMemcpyHostToDevice);
    cudaMemcpy(single_batch_->values_dev, &val, sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    launch_update_kernel(map_, single_batch_);
}

void MapController::slide(float dx, float dy) {
    launch_slide_kernel(map_, dx, dy);
}

void MapController::save_map(const char* filename) {
    save_grid_to_file(map_, filename);
}