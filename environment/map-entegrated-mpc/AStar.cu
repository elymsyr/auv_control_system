#include <cmath>
#include <vector>
#include <cuda_runtime.h>
#include "AStar.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>


#define CHECK_CUDA(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        fprintf(stderr, "CUDA error at %s:%d code=%d(%s)\n", \
            __FILE__, __LINE__, err, cudaGetErrorString(err)); \
        exit(EXIT_FAILURE); \
    } \
}

AStar::AStar(int width, int height) 
    : width_(width), height_(height) {
    start_x_ = width / 2;
    start_y_ = height / 2;
    goal_x_ = width / 2;
    goal_y_ = height / 2;

    CHECK_CUDA(cudaMalloc(&d_grid_, width_ * height_ * sizeof(Node)));
    initializeGrid();
}

AStar::~AStar() {
    CHECK_CUDA(cudaFree(d_grid_));
}

__global__ void initKernel(Node* grid, int width, int height) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int idy = blockIdx.y * blockDim.y + threadIdx.y;
    if (idx >= width || idy >= height) return;

    int index = idy * width + idx;
    Node* node = &grid[index];
    node->x = idx;
    node->y = idy;
    node->g = sqrtf(powf(idy, 2) + powf(idx, 2));
    node->h = 0;
    node->f = 1e9;
    node->parent_x = -1;
    node->parent_y = -1;
    node->status = 0;
}

void AStar::initializeGrid() {
    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);

    initKernel<<<grid, block>>>(d_grid_, width_, height_);
    CHECK_CUDA(cudaDeviceSynchronize());
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
    node->f = node->g + node->h;
}

void AStar::setGoal(int x, int y) {
    goal_x_ = x;
    goal_y_ = y;

    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);

    setGoalKernel<<<grid, block>>>(d_grid_, width_, height_, goal_x_, goal_y_);
    CHECK_CUDA(cudaDeviceSynchronize());
}

__global__ void aStarKernel(Node* grid, uint8_t* obstacles, int width, int height, 
                            int start_x, int start_y, int goal_x, int goal_y) {
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
        if (curr->status != 1) continue; // only process open nodes

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

__global__ void computePathLength(Node* grid, int width, int height, 
                                 int start_x, int start_y, int goal_x, int goal_y, 
                                 int* length) {
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

// Path reconstruction kernel
__global__ void reconstructPath(Node* grid, int width, int height, 
                               int start_x, int start_y, int goal_x, int goal_y, 
                               int2* path, int length) {
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


Path AStar::findPath(uint8_t* obstacles) {
    if (goal_x_ < 0 || goal_y_ < 0) return Path{nullptr, 0};

    // Run A* kernel
    dim3 block(16, 16);
    dim3 grid((width_ + block.x - 1) / block.x, 
              (height_ + block.y - 1) / block.y);
    size_t shared_mem = block.x * block.y * sizeof(bool);
    
    aStarKernel<<<grid, block, shared_mem>>>(d_grid_, obstacles, width_, height_,
        start_x_, start_y_, goal_x_, goal_y_);
    CHECK_CUDA(cudaDeviceSynchronize());

    // Compute path length
    int* d_length;
    CHECK_CUDA(cudaMalloc(&d_length, sizeof(int)));
    computePathLength<<<1,1>>>(d_grid_, width_, height_, 
                              start_x_, start_y_, goal_x_, goal_y_, d_length);
    CHECK_CUDA(cudaDeviceSynchronize());

    int length;
    CHECK_CUDA(cudaMemcpy(&length, d_length, sizeof(int), cudaMemcpyDeviceToHost));
    CHECK_CUDA(cudaFree(d_length));

    Path path_result = {nullptr, 0};
    if (length <= 0) return path_result;

    // Allocate and reconstruct path
    CHECK_CUDA(cudaMalloc(&path_result.points, length * sizeof(int2)));
    reconstructPath<<<1,1>>>(d_grid_, width_, height_, 
                            start_x_, start_y_, goal_x_, goal_y_, 
                            path_result.points, length);
    CHECK_CUDA(cudaDeviceSynchronize());
    
    path_result.length = length;
    return path_result;
}

void AStar::freePath(Path path) {
    if (path.points) {
        CHECK_CUDA(cudaFree(path.points));
    }
}