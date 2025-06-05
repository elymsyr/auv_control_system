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
    
    // Test neural network simulation
    {
        std::cout << "\n--- Neural Network Test ---\n";

        // Add some obstacles
        map.updateSinglePoint(0.0f, 0.0f, 122);  // Near vehicle
        map.updateSinglePoint(10.0f, 10.0f, 122);  // Near vehicle
        map.updateSinglePoint(30.0f, 30.0f, 122);  // Near vehicle
        map.updateSinglePoint(100.0f, 100.0f, 122);  // Far away
        map.updateSinglePoint(50.0f, 50.0f, 122);   // On path
        map.updateSinglePoint(70.0f, 30.0f, 122);   // Off path
        
        // Save initial map
        map.save("initial_map.bin");
        std::cout << "Initial map saved to initial_map.bin\n";
        simulate_neural_network(map.getGridDevicePtr(), WIDTH, HEIGHT);
    }

    // Test obstacle selection
    {
        std::cout << "\n--- Obstacle Selection Test ---\n";
        
        // Set velocity and reference
        map.set_velocity(1.0f, 1.0f);  // Moving diagonally
        map.set_x_ref(60.0f, 60.0f);    // Reference point
        
        // Select 3 obstacles
        auto obstacles = map.obstacle_selection(3);
        
        std::cout << "Selected obstacles:\n";
        for (size_t i = 0; i < obstacles.size(); i++) {
            std::cout << "  Obstacle " << i+1 << ": (" 
                      << std::fixed << std::setprecision(1) << obstacles[i].first
                      << ", " << obstacles[i].second << ")\n";
        }
    }
    
    // Test map sliding
    {
        std::cout << "\n--- Map Sliding Test ---\n";
        
        // Slide map (simulate vehicle movement)
        map.save("shift_wait.bin");
        map.slide(122.0f, -47.0f);
        std::cout << "Map slid by (122.0, -47.0)\n";
        map.save("shift_shifted.bin");
        
        // Add new obstacle after sliding
        map.updateSinglePoint(40.0f, 40.0f, 255);
        std::cout << "Added obstacle at (40.0, 40.0)\n";
        map.save("shift_updated.bin");
        // Save updated map
        std::cout << "Updated map saved to slid_map.bin\n";
        
        // Test obstacle selection again
        auto obstacles = map.obstacle_selection(2);
        std::cout << "Obstacles after sliding:\n";
        for (size_t i = 0; i < obstacles.size(); i++) {
            std::cout << "  Obstacle " << i+1 << ": (" 
                      << obstacles[i].first << ", " << obstacles[i].second << ")\n";
        }
    }
    
    // Test point batch operations
    {
        std::cout << "\n--- Point Batch Test ---\n";
        
        // Create a point batch
        PointBatch* batch = EnvironmentMap::createPointBatch(5);
        
        // Fill with random points
        EnvironmentMap::fillPointBatchWithRandom(batch, WIDTH, HEIGHT);
        
        // Update map with batch
        map.updateWithBatch(batch);
        std::cout << "Updated map with 5 random points\n";
        
        // Clean up
        EnvironmentMap::destroyPointBatch(batch);
        
        // Save final map
        map.save("final_map.bin");
        std::cout << "Final map saved to final_map.bin\n";
    }
    
    // Test xref calculation
    {
        std::cout << "\n--- Xref Calculation Test ---\n";
        float* xref = calculate_xref(&map, 1, 0);
        if (xref) {
            std::cout << "Xref values: ";
            for (int i = 0; i < 12; i++) {
                std::cout << xref[i] << " ";
            }
            std::cout << "\n";
            free(xref);
        }
    }
    
    // Test GPU obstacle selection performance
    {
        std::cout << "\n--- GPU Obstacle Selection Performance ---\n";
        
        // Set velocity and reference
        map.set_velocity(1.0f, 1.0f);
        map.set_x_ref(60.0f, 60.0f);
        
        // Time the selection
        auto start = std::chrono::high_resolution_clock::now();
        auto obstacles = map.obstacle_selection(5);
        auto end = std::chrono::high_resolution_clock::now();
        
        std::cout << "Selected " << obstacles.size() << " obstacles in "
                  << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
                  << "μs\n";
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