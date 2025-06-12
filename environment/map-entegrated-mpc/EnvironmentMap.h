#ifndef ENVIRONMENT_MAP_H
#define ENVIRONMENT_MAP_H

#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <utility>
#include <cstdint>

struct PointBatch {
    int count;
    float2* coords_dev;
    uint8_t* values_dev;
};

class EnvironmentMap {
public:
    EnvironmentMap(int width, int height);
    ~EnvironmentMap();

    // Core map operations
    void slide(float dx, float dy);
    void updateWithBatch(class PointBatch* batch);
    void updateSinglePoint(float world_x, float world_y, uint8_t value);
    
    // Neural network utilities
    uint8_t* getGridDevicePtr() const;
    void copyGridToHost(uint8_t* host_buffer) const;
    void save(const char* filename) const;

    // Obstacle selection
    std::vector<std::pair<float, float>> obstacle_selection(int number_obs = 0);
    void set_x_ref(float x, float y);
    void set_velocity(float vx, float vy);

    // Point batch management
    static class PointBatch* createPointBatch(int count);
    static void destroyPointBatch(class PointBatch* batch);
    static void fillPointBatchWithRandom(class PointBatch* batch, int grid_width, int grid_height);
    void debug_grid_update(float world_x, float world_y);
    void gridToGlobal(int i, int j, float& x, float& y) const;

    // Public member variables for kernel access
    int width_;
    int height_;
    int x_, y_;
    float x_r_, y_r_, r_m_, yaw_;
    int sx_, sy_;
    uint8_t *grid_;
    uint8_t *tempGrid_;
    float ref_x_ = 0.0f;
    float ref_y_ = 0.0f;
    float centre_move_factor_ = 0.0f;
    float circle_radius_ = 10.0f;
    int number_obs_to_feed_ = 50;

private:
    float round_;
    class PointBatch* single_batch_;
    float vx_ = 0.0f;
    float vy_ = 0.0f;
};

#endif