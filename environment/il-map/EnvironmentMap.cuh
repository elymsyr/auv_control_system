#include <cuda_runtime.h>
#include <iostream>

// GPU-side data structure
struct EnvironmentMap {
    int width, height;
    float *grid;     // Device grid
    float *tempGrid; // Temporary grid for sliding

    // Host constructor
    __host__ EnvironmentMap(int w, int h);
    // Host destructor
    __host__ ~EnvironmentMap();

    // Device method: slide the grid by (shiftX, shiftY)
    __device__ void slideGrid(int shiftX, int shiftY);

    // Device method: example computation
    __device__ void iterate(float a, float b, float c, float d);
};

// Kernel wrappers
__global__ void iterateKernel(EnvironmentMap *map, float a, float b, float c, float d);

__global__ void slideGridKernel(EnvironmentMap *map, int shiftX, int shiftY);
