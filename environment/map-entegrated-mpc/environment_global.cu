#include "environment.h"
#include <chrono>
#include <algorithm>
#include <tuple>
#include <stdio.h>
#include <cuda_runtime.h>
#include <cfloat>

// Map
__global__ void slidePhase1(uint8_t* grid, uint8_t* tempGrid, int width, int height, int2 shift) {
    int tx = blockIdx.x * blockDim.x + threadIdx.x;
    int ty = blockIdx.y * blockDim.y + threadIdx.y;
    if (tx >= width || ty >= height) return;

    int dst_idx = ty * width + tx;
    int src_x = tx + shift.x;
    int src_y = ty + shift.y;

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

__global__ void pointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m, float2* coords_dev, uint8_t* values_dev, int count) {
    unsigned tid = blockIdx.x * blockDim.x + threadIdx.x;
    if (tid >= count) return;

    float2 coord = coords_dev[tid];
    uint8_t val = values_dev[tid];

    int x_coor = static_cast<int>((coord.x - x_r) / r_m + width / 2.0f);
    int y_coor = static_cast<int>((coord.y - y_r) / r_m + height / 2.0f);

    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        grid[y_coor * width + x_coor] = val;
    }
}

__global__ void singlePointUpdateKernel(uint8_t* grid, int width, int height, 
                                        float x_r, float y_r, float r_m,
                                        float world_x, float world_y, 
                                        uint8_t value, int radius, char mode) {
    if (mode == 'w') {
        world_x = (world_x - x_r) / r_m + width / 2.0f;
        world_y = (world_y - y_r) / r_m + height / 2.0f;
    }

    int x_coor = __float2int_rd(world_x);
    int y_coor = __float2int_rd(world_y);

    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        int index = y_coor * width + x_coor;
        grid[index] = value;
    }

    if (value >= 250) {
        for (int di = -radius; di <= radius; di++) {
            for (int dj = -radius; dj <= radius; dj++) {
                // Calculate squared distance to avoid sqrt
                if (di * di + dj * dj <= radius * radius) {
                    int ni = y_coor + di;
                    int nj = x_coor + dj;
                    if (ni >= 0 && ni < height && nj >= 0 && nj < width) {
                        int n_index = ni * width + nj;
                        if (grid[n_index] < 250) {
                            grid[n_index] = value;
                        }
                    }
                }
            }
        }
    }
}

__global__ void obstacleSelectionKernel(uint8_t* grid, int width, int height, float wx, float wy, float* output_dists, float2* output_coords, int* output_count, int max_output, float circle_radius, float r_m_) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    
    if (idx >= width || idy >= height) return;
    
    int grid_idx = idy * width + idx;
    int cx = width / 2;
    int cy = height / 2;
    float dx = (float)(idx - cx);
    float dy = (float)(idy - cy);
    float dist = sqrtf(dx * dx + dy * dy) * r_m_;
    if (grid[grid_idx] >= 250 && (dist < circle_radius)) {
        int pos = atomicAdd(output_count, 1);
        if (pos < max_output) {
            output_dists[pos] = dist;
            output_coords[pos] = make_float2(wx + dx * r_m_, wy + dy * r_m_);
        }
        return;
    }
}

// A*
__device__ float atomicMinFloat(float* address, float val) {
    int* address_as_i = (int*)address;
    int old = *address_as_i;
    int expected;
    float old_val;
    do {
        expected = old;
        old_val = __int_as_float(expected);
        if (old_val <= val) return old_val;  // Return early if no update needed
        old = atomicCAS(address_as_i, expected, __float_as_int(val));
    } while (expected != old);
    return old_val;
}

__global__ void initKernel(Node* grid, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    node->x = idx;
    node->y = idy;
    node->g = FLT_MAX;
    node->h = FLT_MAX;
    node->parent_x = -1;
    node->parent_y = -1;
    node->status = 1;
}

__global__ void resetGridKernel(Node* grid, uint8_t* map, int width, int height, int goal_x, int goal_y) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    
    node->parent_x = -1;
    node->parent_y = -1;
    
    if (idx == goal_x && idy == goal_y) {
        node->g = 0.0f;  // Goal cost is 0
        node->status = 1; // Goal is traversable
    } else {
        node->g = FLT_MAX;
        node->status = (map[index] >= 250) ? 0 : 1;
    }
    node->h = sqrtf(powf(idx - goal_x, 2) + powf(idy - goal_y, 2));
}

__global__ void wavefrontKernel(Node* grid, int width, int height, int* d_updated) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;
    
    int index = idy * width + idx;
    Node* node = &grid[index];
    
    // Only process free nodes that aren't the goal
    if (node->status != 1 || node->g == 0.0f) return;

    float min_g = FLT_MAX;
    int best_px = -1;
    int best_py = -1;

    // Check 8 neighbors
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            
            int nx = idx + dx;
            int ny = idy + dy;
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
            
            int nidx = ny * width + nx;
            Node* neighbor = &grid[nidx];
            if (neighbor->status != 1) continue;  // Skip obstacles
            
            // Skip unvisited neighbors
            if (neighbor->g == FLT_MAX) continue;

            float cost = (dx != 0 && dy != 0) ? 1.4142f : 1.0f;
            float new_g = neighbor->g + cost;
            
            if (new_g < min_g) {
                min_g = new_g;
                best_px = nx;
                best_py = ny;
            }
        }
    }

    // Update if we found a better path
    if (min_g < node->g) {
        node->g = min_g;
        node->parent_x = best_px;
        node->parent_y = best_py;
        atomicOr(d_updated, 1);  // Mark update
    }
}

__global__ void reconstructPathKernel(Node* grid, int2* path, int* path_length, 
                                     int start_x, int start_y, int goal_x, int goal_y, 
                                     int width) {
    int x = start_x;
    int y = start_y;
    int count = 0;
    int max_length = width * width;  // Safeguard

    // Follow parent pointers from start to goal
    while (x != goal_x || y != goal_y) {
        if (count >= max_length) break;
        path[count++] = make_int2(x, y);
        
        int idx = y * width + x;
        int px = grid[idx].parent_x;
        int py = grid[idx].parent_y;
        
        if (px == -1 || py == -1) break;
        x = px;
        y = py;
    }
    
    // Add final goal point if reached
    if (x == goal_x && y == goal_y) {
        path[count++] = make_int2(x, y);
    }
    *path_length = count;
}