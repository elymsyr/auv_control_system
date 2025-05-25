#include <cuda_runtime.h>
#include <iostream>

// Host constructor
__host__ EnvironmentMap::EnvironmentMap(int w, int h) : width(w), height(h) {
    size_t N = static_cast<size_t>(w) * h;
    cudaMalloc(&grid, N * sizeof(float));
    cudaMalloc(&tempGrid, N * sizeof(float));
    cudaMemset(grid, 0, N * sizeof(float));
}

// Host destructor
__host__ EnvironmentMap::~EnvironmentMap() {
    cudaFree(grid);
    cudaFree(tempGrid);
}

// Device method: slide the grid by (shiftX, shiftY)
__device__ void EnvironmentMap::slideGrid(int shiftX, int shiftY) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    // Calculate source coordinates
    int srcX = tx - shiftX;
    int srcY = ty - shiftY;

    // Boundary check for source
    bool valid = (srcX >= 0 && srcX < width && srcY >= 0 && srcY < height);
    int dstIdx = ty * width + tx;
    int srcIdx = valid ? (srcY * width + srcX) : -1;

    // Copy from source to temporary grid
    if (valid) {
        tempGrid[dstIdx] = grid[srcIdx];
    } else {
        tempGrid[dstIdx] = 0.0f; // Fill with default value
    }

    // Wait for all threads to finish writing to tempGrid
    __syncthreads();

    // Copy tempGrid back to grid
    grid[dstIdx] = tempGrid[dstIdx];
}

// Device method: example computation
__device__ void EnvironmentMap::iterate(float a, float b, float c, float d) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx < width && ty < height) {
        int idx = ty * width + tx;
        grid[idx] = a * grid[idx] + b * tx + c * ty + d;
    }
}

// Kernel wrappers
__global__ void iterateKernel(EnvironmentMap *map, float a, float b, float c, float d) {
    map->iterate(a, b, c, d);
}

__global__ void slideGridKernel(EnvironmentMap *map, int shiftX, int shiftY) {
    map->slideGrid(shiftX, shiftY);
}
