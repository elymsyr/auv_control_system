#include "EnvironmentMap.h"
#include <cuda_runtime.h>
#include <cstdint>
#include <fstream>      // Add for file operations
#include <vector_types.h>  // Add for int2

__host__ void EnvironmentMap::initialize(int w, int h) {
    width = w;
    height = h;
    x_ = y_ = yaw_ = 0.0f;
    x_r_ = y_r_ = 0.0f;
    sx_ = sy_ = 0;
    x_r_cm_ = 25.0f;
    y_r_cm_ = 25.0f;
    round_ = 2 * M_PI;
    size_t size = w * h * sizeof(uint8_t);
    cudaMallocManaged(&grid, size);  // Unified Memory
    cudaMallocManaged(&tempGrid, size);
    cudaMemset(grid, 0, size);
}

__host__ void EnvironmentMap::cleanup() {
    cudaFree(grid);
    cudaFree(tempGrid);
}

__host__ EnvironmentMap::EnvironmentMap(int w, int h)
    : width(w), height(h),
      sx_(0), sy_(0), x_r_(0.0f), y_r_(0.0f) {  // Initialize new members
    size_t size = w * h * sizeof(uint8_t);
    cudaMalloc(&grid, size);
    cudaMalloc(&tempGrid, size);
    cudaMemset(grid, 0, size);
}

__host__ EnvironmentMap::~EnvironmentMap() {
    cudaFree(grid);
    cudaFree(tempGrid);
}

__host__ void EnvironmentMap::applyBatchUpdate(const PointBatch& batch) {
    const int blockSize = 256;
    const int gridSize = (batch.count + blockSize - 1) / blockSize;
    
    pointUpdateKernel<<<gridSize, blockSize>>>(this, batch);
    cudaDeviceSynchronize();
}

__device__ void EnvironmentMap::iterate(float dx, float dy) {
    x_ += dx;
    y_ += dy;
    x_r_ += dx;
    y_r_ += dy;
    if (yaw_ > round_) {
        yaw_ -= round_;
    }
    sx_ = static_cast<int>(x_ / 25.0f);
    sy_ = static_cast<int>(y_ / 25.0f);
    x_ -= sx_ * 25.0f;
    y_ -= sy_ * 25.0f;
    slideGrid();
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

__global__ void iterateKernel(EnvironmentMap* map, float dx, float dy) {
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

__global__ void pointUpdateKernel(EnvironmentMap* map, PointBatch batch) {
    const unsigned tid = blockIdx.x * blockDim.x + threadIdx.x;
    if(tid >= batch.count) return;
    
    const int2 coord = batch.coords_dev[tid];
    const uint8_t val = batch.values_dev[tid];

    int x_coor = static_cast<int>(round((coord.x - map->x_r_) / map->x_r_cm_ + map->width / 2.0f));
    int y_coor = static_cast<int>(round((coord.y - map->y_r_) / map->y_r_cm_ + map->height / 2.0f));

    // if (tid == 0) {
    //     printf("World coord: (%d, %d) → Grid index: (%d, %d)\n",
    //            coord.x, coord.y, x_coor, y_coor);
    //     printf("x_coor(%d) = round(coord.x(%d) - map->x_r_(%f)) / map->x_r_cm_(%f) + map->width(%d) / 2.0f)",
    //            x_coor, coord.x, map->x_r_, map->x_r_cm_, map->width);
    // }

    if (x_coor >= 0 && x_coor < map->width && y_coor >= 0 && y_coor < map->height) {
        map->grid[y_coor * map->width + x_coor] = val;
    }
}

void save_grid_to_file(void* map, const char* filename) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    uint8_t* h_grid = new uint8_t[d_map->width * d_map->height];
    
    cudaMemcpy(h_grid, d_map->grid, 
              d_map->width * d_map->height * sizeof(uint8_t),
              cudaMemcpyDeviceToHost);
    
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<char*>(h_grid), 
             d_map->width * d_map->height);
    file.close();
    
    delete[] h_grid;
}

// Environment Map Management
void* create_environment_map(int w, int h) {
    EnvironmentMap* map;
    cudaMallocManaged(&map, sizeof(EnvironmentMap));
    map->initialize(w, h);
    return map;
}

void destroy_environment_map(void* map) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    d_map->cleanup();
    cudaFree(d_map);
}

// Point Batch Management
void* create_point_batch(int count) {
    PointBatch* batch;
    cudaMallocManaged(&batch, sizeof(PointBatch));
    cudaMalloc(&batch->coords_dev, count * sizeof(int2));
    cudaMalloc(&batch->values_dev, count * sizeof(uint8_t));
    batch->count = count;
    return batch;
}

void destroy_point_batch(void* batch) {
    PointBatch* d_batch = static_cast<PointBatch*>(batch);
    cudaFree(d_batch->coords_dev);
    cudaFree(d_batch->values_dev);
    cudaFree(d_batch);
}

void launch_slide_kernel(void* map, float dx, float dy) {
    auto start = std::chrono::high_resolution_clock::now();
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    dim3 threads(16, 16);
    dim3 blocks(
        (d_map->width + threads.x - 1) / threads.x,
        (d_map->height + threads.y - 1) / threads.y
    );
    iterateKernel<<<blocks, threads>>>(d_map, dx, dy);
    cudaDeviceSynchronize();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    // std::cout << "Slide kernel: " << duration.count() << "μs\n";
}

void launch_update_kernel(void* map, void* batch) {
    EnvironmentMap* d_map = static_cast<EnvironmentMap*>(map);
    PointBatch* d_batch = static_cast<PointBatch*>(batch);
    const int blockSize = 256;
    const int gridSize = (d_batch->count + blockSize - 1) / blockSize;
    pointUpdateKernel<<<gridSize, blockSize>>>(d_map, *d_batch);
    cudaDeviceSynchronize();
}