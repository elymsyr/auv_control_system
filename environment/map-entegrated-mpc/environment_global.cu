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
    int src_x = tx - shift.x;
    int src_y = ty - shift.y;

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

__global__ void singlePointUpdateKernel(uint8_t* grid, int width, int height, float x_r, float y_r, float r_m,float world_x, float world_y, uint8_t value) {
    // Convert world coordinates to grid coordinates
    float grid_x = (world_x - x_r) / r_m + width / 2.0f;
    float grid_y = (world_y - y_r) / r_m + height / 2.0f;
    
    int x_coor = __float2int_rd(grid_x);
    int y_coor = __float2int_rd(grid_y);

    // Check bounds and update grid
    if (x_coor >= 0 && x_coor < width && y_coor >= 0 && y_coor < height) {
        int index = y_coor * width + x_coor;
        grid[index] = value;
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
__device__ void atomicMinFloat(float* address, float val) {
    int* address_as_i = (int*)address;
    int old = *address_as_i;
    int expected;
    do {
        expected = old;
        float old_val = __int_as_float(expected);
        float new_val = fminf(old_val, val);
        old = atomicCAS(address_as_i, expected, __float_as_int(new_val));
    } while (expected != old);
}

__global__ void initKernel(Node* grid, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    node->x = idx;
    node->y = idy;
    node->g = 1e9;
    node->h = 0;
    node->f = 1e9;
    node->parent_x = -1;
    node->parent_y = -1;
    node->status = 0;
}

__global__ void setGoalKernel(Node* grid, int width, int height, int goal_x, int goal_y) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    float dx = idx - goal_x;
    float dy = idy - goal_y;
    node->h = sqrtf(dx*dx + dy*dy);
}

__global__ void aStarKernel(Node* grid, uint8_t* obstacles, int width, int height, int start_x, int start_y, int goal_x, int goal_y) {
    extern __shared__ bool changed[];
    int index = threadIdx.x + threadIdx.y * blockDim.x;
    changed[index] = false;

    if (blockIdx.x == 0 && blockIdx.y == 0 && threadIdx.x == 0 && threadIdx.y == 0) {
        int start_idx = start_y * width + start_x;
        Node* start = &grid[start_idx];
        start->g = 0;
        start->f = start->h;
        start->status = 1; // open
        changed[index] = true;
    }
    __syncthreads();

    bool any_changed = true;
    while (any_changed) {
        any_changed = false;
        __syncthreads();

        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        int idy = blockIdx.y * blockDim.y + threadIdx.y;
        if (idx >= width || idy >= height) continue;

        int curr_idx = idy * width + idx;
        Node* curr = &grid[curr_idx];
        if (curr->status != 1) continue;

        if (idx == goal_x && idy == goal_y) {
            return; // goal reached
        }

        int dx[] = {1, -1, 0, 0};
        int dy[] = {0, 0, 1, -1};

        for (int i = 0; i < 4; ++i) {
            int nx = idx + dx[i];
            int ny = idy + dy[i];
            if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

            int neighbor_idx = ny * width + nx;
            if (obstacles[neighbor_idx] >= 250) continue; // obstacle

            Node* neighbor = &grid[neighbor_idx];
            float tentative_g = curr->g + 1.0f;

            if (tentative_g < neighbor->g) {
                neighbor->g = tentative_g;
                neighbor->f = tentative_g + neighbor->h;
                neighbor->parent_x = idx;
                neighbor->parent_y = idy;
                neighbor->status = 1; // open
                changed[index] = true;
            }
        }

        curr->status = 2; // close current node
        __syncthreads();

        for (int i = 0; i < blockDim.x * blockDim.y; ++i) {
            if (changed[i]) {
                any_changed = true;
                break;
            }
        }
    }
}

__global__ void computePathLength(Node* grid, int width, int height, int start_x, int start_y, int goal_x, int goal_y, int* length) {
    if (goal_x == start_x && goal_y == start_y) {
        *length = 1;
        return;
    }

    Node goal_node = grid[goal_y * width + goal_x];
    if (goal_node.parent_x == -1 || goal_node.parent_y == -1) {
        *length = 0;
        return;
    }

    int count = 0;
    int current_x = goal_x;
    int current_y = goal_y;
    bool reached_start = false;

    while (true) {
        count++;
        if (current_x == start_x && current_y == start_y) {
            reached_start = true;
            break;
        }

        Node node = grid[current_y * width + current_x];
        if (node.parent_x == -1 || node.parent_y == -1) {
            break;
        }
        current_x = node.parent_x;
        current_y = node.parent_y;
    }

    *length = reached_start ? count : 0;
}

__global__ void reconstructPath(Node* grid, int width, int height, int start_x, int start_y, int goal_x, int goal_y, int2* path, int length) {
    if (length == 0) return;

    int current_x = goal_x;
    int current_y = goal_y;
    int index = length - 1; // Fill from end (start) to beginning (goal)

    while (index >= 0) {
        path[index].x = current_x;
        path[index].y = current_y;
        index--;

        if (current_x == start_x && current_y == start_y) break;

        Node node = grid[current_y * width + current_x];
        current_x = node.parent_x;
        current_y = node.parent_y;
    }
}
