#include "EnvironmentMap.h"
#include <cuda_runtime.h>
#include <cstdint>

__host__ void EnvironmentMap::initialize(int w, int h) {
    width = w;
    height = h;
    x_ = y_ = yaw_ = 0.0f;
    size_t size = w * h * sizeof(uint8_t);
    cudaMallocManaged(&grid, size);  // Unified Memory
    cudaMallocManaged(&tempGrid, size);
    cudaMemset(grid, 0, size);
}

__host__ void EnvironmentMap::cleanup() {
    cudaFree(grid);
    cudaFree(tempGrid);
}

__host__ EnvironmentMap::EnvironmentMap(int w, int h) : width(w), height(h) {
    size_t size = w * h * sizeof(uint8_t);  // Size for uint8_t
    cudaMalloc(&grid, size);
    cudaMalloc(&tempGrid, size);
    cudaMemset(grid, 0, size);  // Initialize to 0
}

__host__ EnvironmentMap::~EnvironmentMap() {
    cudaFree(grid);
    cudaFree(tempGrid);
}

__host__ void EnvironmentMap::applyBatchUpdate(const PointBatch& batch) {
    const int blockSize = 256;
    const int gridSize = (batch.count + blockSize - 1) / blockSize;
    
    ultraFastUpdateKernel<<<gridSize, blockSize>>>(this, batch);
    cudaDeviceSynchronize();
}

__device__ void EnvironmentMap::iterate(float dx, float dy) {
    x_ += dx;
    y_ += dy;
    sx_ = static_cast<int>(x_ / 25.0f);
    sy_ = static_cast<int>(y_ / 25.0f);
    x_ -= sx_ * 25.0f;
    y_ -= sy_ * 25.0f;
}

__device__ void EnvironmentMap::slideGrid() {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int srcX = tx - sx_;
    int srcY = ty - sy_;
    int dstIdx = ty * width + tx;

    if (srcX >= 0 && srcX < width && srcY >= 0 && srcY < height) {
        int srcIdx = srcY * width + srcX;
        tempGrid[dstIdx] = grid[srcIdx];
    } else {
        tempGrid[dstIdx] = 0.0f;
    }

    __syncthreads();
    grid[dstIdx] = tempGrid[dstIdx];
}

__device__ void EnvironmentMap::setPoint(int x, int y, uint8_t value) {
    if (x >= 0 && x < width && y >= 0 && y < height) {
        grid[y * width + x] = value;
    }
}

__global__ void setPointKernel(EnvironmentMap* map, int x, int y, uint8_t value) {
    map->setPoint(x, y, value);
}

__device__ void EnvironmentMap::iterate(float a, float b, float c, float d) {}

// Kernel wrapper
__global__ void iterateMovementKernel(EnvironmentMap* map, float dx, float dy) {
    map->iterate(dx, dy);
}

__global__ void slideGridKernel(EnvironmentMap* map, int sx_, int sy_) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= map->width || ty >= map->height) return;

    // Phase 1: Copy grid -> tempGrid with shift
    int srcX = tx - sx_;
    int srcY = ty - sy_;
    int dstIdx = ty * map->width + tx;

    if (srcX >= 0 && srcX < map->width && srcY >= 0 && srcY < map->height) {
        map->tempGrid[dstIdx] = map->grid[srcY * map->width + srcX];
    } else {
        map->tempGrid[dstIdx] = 0;  // Use 0 for uint8_t
    }

    // Phase 2: Copy tempGrid -> grid (after all threads complete phase 1)
    __syncthreads();  // Now safe within block
    map->grid[dstIdx] = map->tempGrid[dstIdx];
}

__global__ void ultraFastUpdateKernel(EnvironmentMap* map, PointBatch batch) {
    const unsigned tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= batch.count) return;
    
    const int2 coord = batch.coords_dev[tid];
    const uint8_t val = batch.values_dev[tid];
    
    if(coord.x >=0 && coord.x < map->width && coord.y >=0 && coord.y < map->height) {
        map->grid[coord.y * map->width + coord.x] = val;
    }
}
