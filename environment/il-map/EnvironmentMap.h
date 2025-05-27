// EnvironmentMap.h
#ifndef ENVIRONMENT_MAP_H
#define ENVIRONMENT_MAP_H

#include <cstdint>
#include <vector_types.h>
#include <thrust/device_vector.h>
#include <cmath> 

#ifdef __cplusplus
extern "C" {
#endif

// C-compatible struct for batch updates
struct PointBatch {
    int count;
    int2* coords_dev;
    uint8_t* values_dev;
};

// C-compatible function declarations
void* create_environment_map(int w, int h);
void destroy_environment_map(void* map);
void* create_point_batch(int count);
void destroy_point_batch(void* batch);
void launch_slide_kernel(void* map, float dx, float dy);
void launch_update_kernel(void* map, void* batch);
void save_grid_to_file(void* map, const char* filename);
void process_batches_with_slide(void* map, void** batches, int num_batches, float dx, float dy);

#ifdef __cplusplus
}
#endif

// CUDA-specific declarations
class EnvironmentMap {
public:
    int width, height;
    int sx_, sy_;
    float x_, y_, yaw_;
    float x_r_, y_r_;
    float x_r_cm_, y_r_cm_;
    float round_;
    uint8_t* grid;
    uint8_t* tempGrid;

    EnvironmentMap(int w, int h);
    __host__ ~EnvironmentMap();
    __host__ void initialize(int w, int h);
    __host__ void cleanup();
    __device__ void iterate(float dx, float dy);
    __host__ void applyBatchUpdate(const PointBatch& batch);
    __device__ void slideGrid();
    __device__ void setPoint(int x, int y, uint8_t value);
};

// Kernel declarations
__global__ void iterateKernel(EnvironmentMap* map, float dx, float dy);
__global__ void slideGridKernel(EnvironmentMap* map, int shiftX, int shiftY);
__global__ void setPointKernel(EnvironmentMap* map, int x, int y, uint8_t value);
__global__ void pointUpdateKernel(EnvironmentMap* map, const PointBatch batch);

#endif // ENVIRONMENT_MAP_H