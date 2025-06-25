#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <cuda_runtime.h>

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

// Performance metrics structure
struct PerformanceMetrics {
    double map_update_time = 0;
    double slide_time = 0;
    double pathfinding_time = 0;
    double mpc_time = 0;
    double total_time = 0;
    size_t gpu_memory_usage = 0;
    
    void reset() {
        map_update_time = 0;
        slide_time = 0;
        pathfinding_time = 0;
        mpc_time = 0;
        total_time = 0;
        gpu_memory_usage = 0;
    }
    
    void print() const {
        std::cout << "\n=== PERFORMANCE METRICS ===\n";
        std::cout << "Map Update:    " << map_update_time << " ms\n";
        std::cout << "Slide:         " << slide_time << " ms\n";
        std::cout << "Pathfinding:   " << pathfinding_time << " ms\n";
        std::cout << "MPC Solve:     " << mpc_time << " ms\n";
        std::cout << "Total:         " << total_time << " ms\n";
        std::cout << "GPU Memory:    " << gpu_memory_usage / (1024 * 1024) << " MB\n";
        std::cout << "===========================\n";
    }
};

// Function to get current GPU memory usage
size_t get_gpu_memory_usage() {
    size_t free, total;
    CUDA_CALL(cudaMemGetInfo(&free, &total));
    return total - free;
}