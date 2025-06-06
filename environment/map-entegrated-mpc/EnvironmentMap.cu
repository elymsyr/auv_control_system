#include "EnvironmentMap.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

// Device function to compute point to line distance
__device__ float point_to_line_distance(float px, float py, 
                                      float x1, float y1, 
                                      float x2, float y2) {
    // Vector from point to line start
    float dx = px - x1;
    float dy = py - y1;
    
    // Vector representing the line
    float line_dx = x2 - x1;
    float line_dy = y2 - y1;
    
    // Length of line segment squared
    float length_sq = line_dx * line_dx + line_dy * line_dy;
    
    // Calculate projection scalar
    float t = (dx * line_dx + dy * line_dy) / (length_sq + 1e-6f);
    t = fmaxf(0.0f, fminf(1.0f, t));  // Clamp to segment
    
    // Calculate closest point on line
    float proj_x = x1 + t * line_dx;
    float proj_y = y1 + t * line_dy;
    
    // Return distance to closest point
    return sqrtf((px - proj_x) * (px - proj_x) + 
                (py - proj_y) * (py - proj_y));
}

// Kernel for obstacle selection
__global__ void obstacleSelectionKernel(uint8_t* grid, int width, int height, 
                                      float current_x, float current_y,
                                      float projected_x, float projected_y,
                                      float x_r, float y_r, float x_r_cm, float y_r_cm,
                                      float* dists, float2* coords, int* count, int max_candidates) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (idx >= width || idy >= height) return;
    
    int grid_idx = idy * width + idx;
    if (grid[grid_idx] > 0) {  // Obstacle threshold TEST (200)
        // Convert grid coordinates to world coordinates
        float world_x = x_r + (idx - width/2.0f) * x_r_cm;
        float world_y = y_r + (idy - height/2.0f) * y_r_cm;
        
        // Calculate distance to path
        float dist = point_to_line_distance(world_x, world_y, 
                                          current_x, current_y,
                                          projected_x, projected_y);
        
        // Get position in candidate array
        int pos = atomicAdd(count, 1);
        if (pos < max_candidates) {
            dists[pos] = dist;
            coords[pos] = make_float2(world_x, world_y);
        }
    }
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

std::vector<std::pair<float, float>> EnvironmentMap::obstacle_selection(int number_obs) {
    const int MAX_CANDIDATES = 10000;
    
    // Device memory for candidates
    float* d_dists;
    float2* d_coords;
    int* d_count;
    
    cudaMalloc(&d_dists, MAX_CANDIDATES * sizeof(float));
    cudaMalloc(&d_coords, MAX_CANDIDATES * sizeof(float2));
    cudaMalloc(&d_count, sizeof(int));
    cudaMemset(d_count, 0, sizeof(int));
    
    // Calculate projected position
    float current_x = x_r_;
    float current_y = y_r_;
    float projected_x, projected_y;
    
    if (ref_x_ != 0.0f || ref_y_ != 0.0f) {
        projected_x = ref_x_;
        projected_y = ref_y_;
    } else {
        projected_x = current_x + vx_ * 2.0f;  // 2 seconds lookahead
        projected_y = current_y + vy_ * 2.0f;
    }
    
    // Launch kernel
    dim3 threads(16, 16);
    dim3 blocks((width_ + threads.x - 1) / threads.x,
                (height_ + threads.y - 1) / threads.y);
    
    obstacleSelectionKernel<<<blocks, threads>>>(
        grid_, width_, height_,
        current_x, current_y,
        projected_x, projected_y,
        x_r_, y_r_, x_r_cm_, y_r_cm_,
        d_dists, d_coords, d_count, MAX_CANDIDATES
    );
    cudaDeviceSynchronize();
    
    // Copy count back to host
    int h_count;
    cudaMemcpy(&h_count, d_count, sizeof(int), cudaMemcpyDeviceToHost);
    h_count = (h_count < MAX_CANDIDATES) ? h_count : MAX_CANDIDATES;
    
    // Copy candidate data
    std::vector<float> h_dists(h_count);
    std::vector<float2> h_coords(h_count);
    
    if (h_count > 0) {
        cudaMemcpy(h_dists.data(), d_dists, h_count * sizeof(float), cudaMemcpyDeviceToHost);
        cudaMemcpy(h_coords.data(), d_coords, h_count * sizeof(float2), cudaMemcpyDeviceToHost);
    }
    
    // Cleanup device memory
    cudaFree(d_dists);
    cudaFree(d_coords);
    cudaFree(d_count);
    
    // Create vector of pairs for sorting
    std::vector<std::pair<float, float2>> candidates;
    for (int i = 0; i < h_count; i++) {
        candidates.push_back({h_dists[i], h_coords[i]});
    }
    
    // Sort by distance
    std::sort(candidates.begin(), candidates.end(), 
        [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    
    // Prepare selected obstacles
    std::vector<std::pair<float, float>> selected;
    int count = (candidates.size() < static_cast<size_t>(number_obs)) ? 
                candidates.size() : number_obs;
    
    for (int i = 0; i < count; i++) {
        selected.push_back({candidates[i].second.x, candidates[i].second.y});
    }
    
    // Pad with distant obstacles if needed
    for (int i = count; i < number_obs; i++) {
        selected.push_back({10000.0f, 10000.0f});  // Far away
    }
    
    return selected;
}

// Utility functions
extern "C" float* calculate_xref(EnvironmentMap* map, int mission, int state) {
    constexpr int LENGTH = 12;
    float* result = static_cast<float*>(malloc(sizeof(float) * LENGTH));
    if (!result) return nullptr;

    for (int i = 0; i < LENGTH; ++i) {
        if (i < 3) result[i] = 10.0f;
        else if (i < 6) result[i] = 0.1f;
        else if (i < 8) result[i] = 2.0f;
        else if (i < 9) result[i] = 1.0f;
        else result[i] = 0.0f;
    }
    return result;
}

void simulate_neural_network(uint8_t* grid_data, int width, int height) {
    std::vector<uint8_t> host_grid(width * height);
    cudaMemcpy(host_grid.data(), grid_data, 
               width * height * sizeof(uint8_t), 
               cudaMemcpyDeviceToHost);
    
    uint8_t max_val = 0;
    for (int i = 0; i < width * height; i++) {
        if (host_grid[i] > max_val) max_val = host_grid[i];
    }
    std::cout << "Max grid value: " << static_cast<int>(max_val) << std::endl;
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

void EnvironmentMap::print_grid_info() const {
    std::cout << "\n=== Grid Information ===\n";
    std::cout << "Dimensions: " << width_ << " x " << height_ << "\n";
    std::cout << "World reference: (" << x_r_ << ", " << y_r_ << ")\n";
    std::cout << "CM per cell: (" << x_r_cm_ << ", " << y_r_cm_ << ")\n";
    std::cout << "Slide offset: (" << sx_ << ", " << sy_ << ")\n";
    
    uint8_t* h_grid = new uint8_t[width_ * height_];
    copyGridToHost(h_grid);
    
    bool all_zeros = true;
    for (int i = 0; i < width_ * height_; i++) {
        if (h_grid[i] != 0) {
            all_zeros = false;
            break;
        }
    }
    
    std::cout << "Grid is all zeros: " << (all_zeros ? "YES" : "NO") << "\n";
    delete[] h_grid;
    std::cout << "========================\n";
}