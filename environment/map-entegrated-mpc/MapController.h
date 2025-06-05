#ifndef MAP_CONTROLLER_H
#define MAP_CONTROLLER_H

#include "EnvironmentMap.h"
#include <cuda_runtime.h>

class MapController {
private:
    EnvironmentMap* map_;
    PointBatch* single_batch_;
    const int width_;
    const int height_;

public:
    MapController(int width, int height);
    ~MapController();

    // Grid access for neural network
    uint8_t* get_grid_device_ptr() const;
    void copy_grid_to_host(uint8_t* host_buffer) const;
    
    // Map operations
    void process_batches_with_slide(void** batches, int num_batches, float dx, float dy);
    static void fill_point_batch_with_random(void* batch, int grid_width, int grid_height);
    void initialize_test_pattern();
    void update_single_point(float world_x, float world_y, uint8_t value);
    void slide(float dx, float dy);
    void save_map(const char* filename);
    EnvironmentMap* get_map() const { return map_; }
    int get_width() const { return width_; }
    int get_height() const { return height_; }
};

#endif