#include "MapController.h"
#include "EnvironmentMap.h"
#include "mpc.h"
#include <iostream>
#include <vector>

// Simple neural network simulator (would be replaced with actual NN)
void simulate_neural_network(uint8_t* grid_data, int width, int height) {
    std::cout << "Neural network processing grid of size: " 
              << width << "x" << height << std::endl;
    
    // In a real implementation, this would run on GPU
    std::vector<uint8_t> host_grid(width * height);
    cudaMemcpy(host_grid.data(), grid_data, width * height * sizeof(uint8_t), 
               cudaMemcpyDeviceToHost);
    
    // Simple analysis - find max value
    uint8_t max_val = 0;
    for (int i = 0; i < width * height; i++) {
        if (host_grid[i] > max_val) max_val = host_grid[i];
    }
    std::cout << "Max grid value: " << static_cast<int>(max_val) << std::endl;
}

int main() {
    const int W = 129, H = 129;
    
    // 1. Create controller
    MapController controller(W, H);
    
    // 2. Create and fill batches
    void* random_batch = create_point_batch(100);
    MapController::fill_point_batch_with_random(random_batch, W, H);
    
    void* sensor_batch = create_point_batch(150);
    MapController::fill_point_batch_with_random(sensor_batch, W, H);

    // 3. Update map with batches
    void* batches[] = {random_batch, sensor_batch};
    controller.process_batches_with_slide(batches, 2, 0, 0);
    
    // 4. Test single point update
    controller.update_single_point(322.0f, 123.0f, 255);
    
    // 5. Test test pattern
    controller.initialize_test_pattern();
    controller.save_map("test_initial.bin");
    
    // 6. Test slide
    controller.slide(56.0f, 30.5f);
    controller.save_map("test_shifted.bin");
    
    // 7. Test neural network access
    uint8_t* grid_device_ptr = controller.get_grid_device_ptr();
    simulate_neural_network(grid_device_ptr, W, H);
    
    // 8. MPC integration test
    VehicleModel model("config.json");
    NonlinearMPC mpc(model, 20, 0.1);
    
    bool running = true;
    int iteration = 0;
    while (running && iteration < 3) {  // Limit to 3 iterations for testing
        std::cout << "\n--- Control Cycle " << iteration + 1 << " ---" << std::endl;
        
        // Update with new sensor data
        void* new_sensor_batch = create_point_batch(50);
        MapController::fill_point_batch_with_random(new_sensor_batch, W, H);
        launch_update_kernel(controller.get_map(), new_sensor_batch);
        
        // Process with neural network
        simulate_neural_network(controller.get_grid_device_ptr(), W, H);
        
        // Solve MPC
        DM x0 = DM::zeros(12);
        DM x_ref = DM::repmat(DM::vertcat({10.8, 9.9, -6, 0, 0, 0, 0, 0, 0, 0, 0, 0}), 1, 21);
        auto [control, trajectory] = mpc.solve(x0, x_ref);
        
        std::cout << "MPC Control Output: " << control << std::endl;
        
        // Apply control (simulated)
        float dx = 10.0f * static_cast<float>(iteration);
        float dy = 5.0f * static_cast<float>(iteration);
        controller.slide(dx, dy);
        
        // Cleanup
        destroy_point_batch(new_sensor_batch);
        iteration++;
    }
    
    // Final cleanup
    destroy_point_batch(random_batch);
    destroy_point_batch(sensor_batch);
    
    std::cout << "MapController testing completed successfully." << std::endl;
    return 0;
}

// # Clean up previous artifacts
// rm -f *.o *.so main jit_* libdynamics_func*

// # 1. Compile CUDA environment
// nvcc -arch=sm_75 -c EnvironmentMap.cu -o EnvironmentMap.o

// # 2. Compile main.cpp with NVCC (use -dc for device code linking)
// nvcc -arch=sm_75 -dc -I"${CONDA_PREFIX}/include" main.cpp -o main.o

// # 3. Compile MPC code with CasADi
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c mpc.cpp -o mpc.o

// # 3. Compile Map Controller code
// g++ -std=c++17 -I"${CONDA_PREFIX}/include" -c MapController.cpp -o MapController.o

// # 4. Build barrier function as shared library
// nvcc -arch=sm_75 -Xcompiler -fPIC -shared barrier.cu -o barrier.so

// # 5. Link all objects
// g++ -o main EnvironmentMap.o main.o mpc.o MapController.o -L"${CONDA_PREFIX}/lib" -Wl,-rpath,"${CONDA_PREFIX}/lib" -lcasadi -lipopt -lzmq -lcudart -L/usr/local/cuda/lib64

// # 6. Run
// ./main