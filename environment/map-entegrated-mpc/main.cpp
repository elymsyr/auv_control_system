#include "EnvironmentMap.h"
#include <iostream>
#include <vector>
#include <iomanip>
#include <fstream>
#include <chrono>
#include <cstdlib>
#include <cmath>

#define CUDA_CALL(call) { \
    cudaError_t err = call; \
    if (err != cudaSuccess) { \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ \
                  << ": " << cudaGetErrorString(err) << "\n"; \
        exit(EXIT_FAILURE); \
    } \
}

int main() {
    // Create environment map
    const int WIDTH = 129;
    const int HEIGHT = 129;
    EnvironmentMap map(WIDTH, HEIGHT);
    
    std::cout << "After construction:\n";
    map.print_grid_info();
    // map.save("initial_empty.bin");

    {   
        std::cout << "\n--- Adding Single Points ---\n";

        map.updateSinglePoint(0.0f, 0.0f, 150.0f);
        map.updateSinglePoint(102.0f, 165.0f, 150.0f);
        map.updateSinglePoint(302.0f, 120.0f, 150.0f);
        map.updateSinglePoint(385.0f, 240.0f, 150.0f);
        map.updateSinglePoint(500.0f, 100.0f, 150.0f);
        map.updateSinglePoint(1000.0f, 450.0f, 150.0f);

        map.print_grid_info();
        map.save("initial_updated.bin");
    }

    {
        std::cout << "\n--- Obstacle Selection Test ---\n";
        
        // Set velocity and reference
        map.set_velocity(1.0f, 1.0f);  // Moving diagonally
        map.set_x_ref(325.0f, 130.0f);    // Reference point
        
        // Select 3 obstacles
        auto obstacles = map.obstacle_selection(5);
        
        std::cout << "Selected obstacles:\n";
        for (size_t i = 0; i < obstacles.size(); i++) {
            std::cout << "  Obstacle " << i+1 << ": (" 
                      << std::fixed << std::setprecision(1) << obstacles[i].first
                      << ", " << obstacles[i].second << ")\n";
        }
    }

    // {
    //     std::cout << "\n--- Map Sliding Test ---\n";
        
    //     map.slide(322.0f, 0.0f);
    //     std::cout << "Map slid by (122.0, -47.0)\n";
    //     map.save("initial_shifted.bin");
        
    //     auto obstacles = map.obstacle_selection(5);
    //     std::cout << "Obstacles after sliding:\n";
    //     for (size_t i = 0; i < obstacles.size(); i++) {
    //         std::cout << "  Obstacle " << i+1 << ": (" 
    //                   << obstacles[i].first << ", " << obstacles[i].second << ")\n";
    //     }
    // }

    {
        std::cout << "\n--- Point Batch Test ---\n";
        PointBatch* batch = EnvironmentMap::createPointBatch(2000);
        EnvironmentMap::fillPointBatchWithRandom(batch, WIDTH, HEIGHT);
        map.updateWithBatch(batch);
        EnvironmentMap::destroyPointBatch(batch);
        map.save("initial_filled.bin");
    }

    {
        map.set_velocity(100.0f, 100.0f);
        map.set_x_ref(525.0f, 525.0f);

        auto obstacles = map.obstacle_selection(50);
        std::cout << "Obstacles after sliding:\n";
        for (size_t i = 0; i < obstacles.size(); i++) {
            map.updateSinglePoint(obstacles[i].first, obstacles[i].second, 200);
        }
        map.updateSinglePoint(525.0f, 525.0f, 255);
        map.updateSinglePoint(0.0f, 0.0f, 250);
        map.updateSinglePoint(10.0f, 10.0f, 255);
        map.save("initial_signed.bin");
    }

    return 0;
}

// rm -f *.o *.so main jit_* libdynamics_func* *.bin

// nvcc -arch=sm_75 -c EnvironmentMap.cu -o EnvironmentMap.o

// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" main.cpp -o main.o

// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c mpc.cpp -o mpc.o

// g++ -o main EnvironmentMap.o main.o mpc.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64

// ./main

// rm -f *.o *.so main jit_* libdynamics_func*

// python /home/eren/GitHub/ControlSystem/environment/map-entegrated-mpc/visualize.py