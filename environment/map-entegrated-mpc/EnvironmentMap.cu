#include "EnvironmentMap.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cfloat>

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

// Device functions
__global__ void slidePhase1(uint8_t* grid, uint8_t* tempGrid, int width, int height, int sx, int sy) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int dst_idx = ty * width + tx;
    int src_x = tx - sx;
    int src_y = ty - sy;

    if (src_x >= 0 && src_x < width && src_y >= 0 && src_y < height) {
        tempGrid[dst_idx] = grid[src_y * width + src_x];
    } else {
        tempGrid[dst_idx] = 0;
    }
}

__global__ void slidePhase2(uint8_t* grid, uint8_t* tempGrid, int width, int height) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int idx = ty * width + tx;
    grid[idx] = tempGrid[idx];
}

__global__ void pointUpdateKernel(
    uint8_t* grid, int width, int height,
    float x_r, float y_r, float x_r_cm, float y_r_cm,
    float2* coords_dev, uint8_t* values_dev, int count
) {
    unsigned tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= count) return;

    float2 coord = coords_dev[tid];
    uint8_t val = values_dev[tid];

    int x_coor = static_cast<int>((coord.x - x_r) / x_r_cm + width / 2.0f);
    int y_coor = static_cast<int>((coord.y - y_r) / y_r_cm + height / 2.0f);

    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        grid[y_coor * width + x_coor] = val;
    }
}

__global__ void obstacleSelectionKernel(
    uint8_t* grid, int width, int height, 
    float current_x, float current_y, float middle_x, float middle_y,
    float x_r, float y_r, float x_r_cm, float y_r_cm,
    float* output_dists, float2* output_coords,  float* output_prior,
    int* output_count, int max_output,
    float circle_radius, float centre_move_factor
) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (idx >= width || idy >= height) return;
    
    int grid_idx = idy * width + idx;
    if (grid[grid_idx] > 0) { // TEST
        // Convert to world coordinates
        float world_x = x_r + (idx - width/2.0f) * x_r_cm;
        float world_y = y_r + (idy - height/2.0f) * y_r_cm;

        float dx = world_x - current_x;
        float dy = world_y - current_y;
        float dist_to_current = sqrtf(dx*dx + dy*dy);

        dx = world_x - middle_x;
        dy = world_y - middle_y;
        float dist_to_middle = sqrtf(dx*dx + dy*dy);

        // Priority 1: Check if in circle
        if (dist_to_current <= circle_radius) {
            
            // Add to output
            int pos = atomicAdd(output_count, 1);
            if (pos < max_output) {
                output_dists[pos] = dist_to_current;
                output_coords[pos] = make_float2(world_x, world_y);
                output_prior[pos] = dist_to_middle * 0.2 + dist_to_current * 0.8;
            }
            return;
        }
    }
}

float2 move_to(float x1, float y1, float x2, float y2, float factor) {
    // Calculate direction vector
    float dx = x2 - x1;
    float dy = y2 - y1;
    // Compute the distance to the target point
    float length = std::sqrt(dx * dx + dy * dy);
    // Compute new point
    float proj_x = x1 + (dx / length) * factor;
    float proj_y = y1 + (dy / length) * factor;
    return make_float2(proj_x, proj_y);
}

float distance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<std::pair<float, float>> EnvironmentMap::obstacle_selection(int number_obs) {
    const int MAX_CANDIDATES = 10000;
    const int TOP_K = number_obs > 0 ? number_obs : number_obs_to_feed_;

    // Device memory allocations
    float* d_dists;
    float* d_prior;
    float2* d_coords;
    int* d_count;
    
    CUDA_CALL(cudaMalloc(&d_dists, MAX_CANDIDATES * sizeof(float)));
    CUDA_CALL(cudaMalloc(&d_prior, MAX_CANDIDATES * sizeof(float)));
    CUDA_CALL(cudaMalloc(&d_coords, MAX_CANDIDATES * sizeof(float2)));
    CUDA_CALL(cudaMalloc(&d_count, sizeof(int)));
    CUDA_CALL(cudaMemset(d_count, 0, sizeof(int)));
    
    float current_x = x_r_;
    float current_y = y_r_;
    float ref_x = ref_x_ - current_x;
    float ref_y = ref_y_ - current_y;
    float proj_x = current_x + vx_ * 5.0f;
    float proj_y = current_y + vy_ * 5.0f;

    if (distance(x_r_, y_r_, ref_x, ref_y) > 1000.0f) {
        // Reset reference point if too far
        float2 ref_new = move_to(x_r_, y_r_, ref_x, ref_y, 1000.0f);
        ref_x = ref_new.x;
        ref_y = ref_new.y;
    }
    if (distance(x_r_, y_r_, proj_x, proj_y) > 1000.0f) {
        // Reset reference point if too far
        float2 proj_new = move_to(x_r_, y_r_, proj_x, proj_y, 1000.0f);
        proj_x = proj_new.x;
        proj_y = proj_new.y;
    }

    float middle_x = (proj_x + ref_x) / 2.0f;
    float middle_y = (proj_y + ref_y) / 2.0f;

    if (middle_x > 0.0f || middle_y > 0.0f) {
        // Move middle point towards the reference point
        float2 new_current = move_to(current_x, current_y, middle_x, middle_y, centre_move_factor_);
        current_x = new_current.x;
        current_y = new_current.y;
    }

    // Launch obstacle selection kernel
    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);

    obstacleSelectionKernel<<<blocks, threads>>>(
        grid_, width_, height_,
        current_x, current_y, middle_x, middle_y,
        x_r_, y_r_, x_r_cm_, y_r_cm_,
        d_dists, d_coords, d_prior, d_count, MAX_CANDIDATES, circle_radius_, centre_move_factor_
    );
    CUDA_CALL(cudaDeviceSynchronize());
    
    // Get candidate count
    int h_count;
    CUDA_CALL(cudaMemcpy(&h_count, d_count, sizeof(int), cudaMemcpyDeviceToHost));
    h_count = min(h_count, MAX_CANDIDATES);
    
    // Early return if no obstacles
    if (h_count == 0) {
        std::vector<std::pair<float, float>> result(TOP_K, {10000.0f, 10000.0f});
        CUDA_CALL(cudaFree(d_dists));
        CUDA_CALL(cudaFree(d_coords));
        CUDA_CALL(cudaFree(d_count));
        CUDA_CALL(cudaFree(d_prior));
        return result;
    }
    
    // Copy candidates to host
    std::vector<float> h_prior(h_count);
    std::vector<float2> h_coords(h_count);
    CUDA_CALL(cudaMemcpy(h_prior.data(), d_prior, h_count * sizeof(float), cudaMemcpyDeviceToHost));
    CUDA_CALL(cudaMemcpy(h_coords.data(), d_coords, h_count * sizeof(float2), cudaMemcpyDeviceToHost));
    
    // Create index array and sort by distance to projected point
    std::vector<int> indices(h_count);
    for (int i = 0; i < h_count; ++i) indices[i] = i;
    
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
        return h_prior[a] < h_prior[b];
    });
    
    // Prepare results with top K closest obstacles
    std::vector<std::pair<float, float>> result;
    int num_to_return = std::min(TOP_K, h_count);
    for (int i = 0; i < num_to_return; ++i) {
        int idx = indices[i];
        result.push_back({h_coords[idx].x, h_coords[idx].y});
    }
    
    // Pad with default values if needed
    for (int i = num_to_return; i < TOP_K; ++i) {
        result.push_back({10000.0f, 10000.0f});
    }
    
    // Cleanup
    CUDA_CALL(cudaFree(d_dists));
    CUDA_CALL(cudaFree(d_coords));
    CUDA_CALL(cudaFree(d_count));
    CUDA_CALL(cudaFree(d_prior));

    return result;
}

// PointBatch management
PointBatch* EnvironmentMap::createPointBatch(int count) {
    PointBatch* batch = new PointBatch;
    batch->count = count;
    cudaMalloc(&batch->coords_dev, count * sizeof(float2));
    cudaMalloc(&batch->values_dev, count * sizeof(uint8_t));
    return batch;
}

void EnvironmentMap::destroyPointBatch(PointBatch* batch) {
    cudaFree(batch->coords_dev);
    cudaFree(batch->values_dev);
    delete batch;
}

void EnvironmentMap::fillPointBatchWithRandom(PointBatch* batch, int grid_width, int grid_height) {
    float2* h_coords = new float2[batch->count];
    uint8_t* h_values = new uint8_t[batch->count];
    
    for(int i = 0; i < batch->count; ++i) {
        h_coords[i].x = (rand() % (grid_width * 25 * 2)) - (grid_width * 25);
        h_coords[i].y = (rand() % (grid_height * 25 * 2)) - (grid_height * 25);
        h_values[i] = rand() % 1 + 50; // TEST
    }
    
    cudaMemcpy(batch->coords_dev, h_coords, 
               batch->count * sizeof(float2), cudaMemcpyHostToDevice);
    cudaMemcpy(batch->values_dev, h_values,
               batch->count * sizeof(uint8_t), cudaMemcpyHostToDevice);
    
    delete[] h_coords;
    delete[] h_values;
}

// EnvironmentMap core methods
EnvironmentMap::EnvironmentMap(int width, int height) 
    : width_(width), height_(height),
      x_(0.0f), y_(0.0f), yaw_(0.0f),
      x_r_(0.0f), y_r_(0.0f),
      sx_(0), sy_(0),
      x_r_cm_(25.0f), y_r_cm_(25.0f),
      round_(2 * M_PI) {
    
    size_t size = width * height * sizeof(uint8_t);
    cudaMalloc(&grid_, size);
    cudaMalloc(&tempGrid_, size);
    cudaMemset(grid_, 0, size);
    single_batch_ = createPointBatch(1);
}

EnvironmentMap::~EnvironmentMap() {
    destroyPointBatch(single_batch_);
    cudaFree(grid_);
    cudaFree(tempGrid_);
}

void EnvironmentMap::slide(float dx, float dy) {
    x_ += dx;
    y_ += dy;
    x_r_ += dx;
    y_r_ += dy;
    
    if (yaw_ > round_) {
        yaw_ -= round_;
    }
    
    // FIX: Remove 'int' declaration to update member variables
    sx_ = static_cast<int>(x_ / x_r_cm_);
    sy_ = static_cast<int>(y_ / y_r_cm_);
    
    x_ -= sx_ * x_r_cm_;
    y_ -= sy_ * y_r_cm_;
    
    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);

    slidePhase1<<<blocks, threads>>>(grid_, tempGrid_, width_, height_, sx_, sy_);
    cudaDeviceSynchronize();
    slidePhase2<<<blocks, threads>>>(grid_, tempGrid_, width_, height_);
    cudaDeviceSynchronize();
}

void EnvironmentMap::updateWithBatch(PointBatch* batch) {
    const int blockSize = 256;
    const int gridSize = (batch->count + blockSize - 1) / blockSize;
    pointUpdateKernel<<<gridSize, blockSize>>>(
        grid_, width_, height_,
        x_r_, y_r_, x_r_cm_, y_r_cm_,
        batch->coords_dev, batch->values_dev, batch->count
    );
    cudaDeviceSynchronize();
}

void EnvironmentMap::updateSinglePoint(float world_x, float world_y, uint8_t value) {
    float2 coord = make_float2(world_x, world_y);
    uint8_t val = value;

    cudaMemcpy(single_batch_->coords_dev, &coord, sizeof(float2), cudaMemcpyHostToDevice);
    cudaMemcpy(single_batch_->values_dev, &val, sizeof(uint8_t), cudaMemcpyHostToDevice);

    updateWithBatch(single_batch_);
}

uint8_t* EnvironmentMap::getGridDevicePtr() const {
    return grid_;
}

void EnvironmentMap::copyGridToHost(uint8_t* host_buffer) const {
    size_t grid_size = width_ * height_ * sizeof(uint8_t);
    cudaMemcpy(host_buffer, grid_, grid_size, cudaMemcpyDeviceToHost);
    cudaDeviceSynchronize();
}

void EnvironmentMap::save(const char* filename) const {
    uint8_t* h_grid = new uint8_t[width_ * height_];
    copyGridToHost(h_grid);
    
    std::ofstream file(filename, std::ios::binary);
    file.write(reinterpret_cast<char*>(h_grid), width_ * height_);
    file.close();
    
    delete[] h_grid;
}

void EnvironmentMap::set_velocity(float vx, float vy) {
    vx_ = vx;
    vy_ = vy;
}

void EnvironmentMap::set_x_ref(float x, float y) {
    ref_x_ = x;
    ref_y_ = y;
}

// Add to EnvironmentMap implementation
void EnvironmentMap::debug_grid_update(float world_x, float world_y) {
    // Calculate expected grid coordinates
    int x_coor = static_cast<int>((world_x - x_r_) / x_r_cm_ + width_ / 2.0f);
    int y_coor = static_cast<int>((world_y - y_r_) / y_r_cm_ + height_ / 2.0f);
    
    std::cout << "=== Grid Update Debug ===\n";
    std::cout << "World coordinates: (" << world_x << ", " << world_y << ")\n";
    std::cout << "x_r_: " << x_r_ << ", y_r_: " << y_r_ << "\n";
    std::cout << "x_r_cm_: " << x_r_cm_ << ", y_r_cm_: " << y_r_cm_ << "\n";
    std::cout << "Calculated grid coordinates: (" << x_coor << ", " << y_coor << ")\n";
    
    // Check if coordinates are valid
    if (x_coor >= 0 && x_coor < width_ && y_coor >= 0 && y_coor < height_) {
        std::cout << "Coordinates are within grid bounds\n";
        
        // Directly set the value using a debug kernel
        uint8_t debug_value = 255;
        
        // Create a debug batch
        PointBatch* debug_batch = createPointBatch(1);
        float2 coord = make_float2(world_x, world_y);
        
        CUDA_CALL(cudaMemcpy(debug_batch->coords_dev, &coord, sizeof(float2), cudaMemcpyHostToDevice));
        CUDA_CALL(cudaMemcpy(debug_batch->values_dev, &debug_value, sizeof(uint8_t), cudaMemcpyHostToDevice));
        
        // Run update kernel
        const int blockSize = 256;
        const int gridSize = (1 + blockSize - 1) / blockSize;
        pointUpdateKernel<<<gridSize, blockSize>>>(
            grid_, width_, height_,
            x_r_, y_r_, x_r_cm_, y_r_cm_,
            debug_batch->coords_dev, debug_batch->values_dev, 1
        );
        CUDA_CALL(cudaDeviceSynchronize());
        
        // Check if value was set
        uint8_t grid_value;
        size_t offset = y_coor * width_ + x_coor;
        CUDA_CALL(cudaMemcpy(&grid_value, grid_ + offset, sizeof(uint8_t), cudaMemcpyDeviceToHost));
        
        std::cout << "Value at (" << x_coor << ", " << y_coor << "): " 
                  << static_cast<int>(grid_value) << "\n";
        
        destroyPointBatch(debug_batch);
    } else {
        std::cout << "Coordinates are OUTSIDE grid bounds!\n";
    }
    std::cout << "========================\n";
}