#ifndef ENVIRONMENT_MAP_CUH
#define ENVIRONMENT_MAP_CUH

#include <cstdint>
#include <vector_types.h>
#include <thrust/device_vector.h>  // At the top of the file

struct PointBatch {
    int count;
    int2* coords_dev;    // Using int2 for coalesced access
    uint8_t* values_dev;
};

class EnvironmentMap {
public:
    int width, height;
    int sx_, sy_;
    float x_, y_, yaw_;
    uint8_t* grid;      // Device memory pointer
    uint8_t* tempGrid;  // Temporary buffer pointer

    __host__ EnvironmentMap(int w, int h);
    __host__ ~EnvironmentMap();
    __host__ void initialize(int w, int h);
    __host__ void cleanup();
    __device__ void iterate(float dx, float dy);
    __host__ void applyBatchUpdate(const PointBatch& batch);
    __device__ void slideGrid();
    __device__ void iterate(float a, float b, float c, float d);
    __device__ void setPoint(int x, int y, uint8_t value);
};

__global__ void iterateMovementKernel(EnvironmentMap* map, float dx, float dy);
__global__ void slideGridKernel(EnvironmentMap* map, int shiftX, int shiftY);
__global__ void setPointKernel(EnvironmentMap* map, int x, int y, uint8_t value);
__global__ void ultraFastUpdateKernel(EnvironmentMap* map, const PointBatch batch);

#endif