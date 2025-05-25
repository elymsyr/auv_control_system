// HostController.cpp
#include "HostController.hpp"
#include "EnvironmentMap.cuh" // GPU class header

__global__ void iterateKernel(EnvironmentMap* map, float a, float b, float c, float d) {
    map->iterate(a, b, c, d);
}

HostEnvironmentMapController::HostEnvironmentMapController(int width, int height) {
    // Allocate GPU memory for EnvironmentMap instance
    cudaMalloc(&d_map, sizeof(EnvironmentMap));
    
    // Initialize GPU object (requires a device constructor)
    EnvironmentMap h_map_init(width, height); // Temporary host object
    cudaMemcpy(d_map, &h_map_init, sizeof(EnvironmentMap), cudaMemcpyHostToDevice);

    // Optional: Allocate host-side buffer for grid data
    h_gridSnapshot = new float[width * height];
}

HostEnvironmentMapController::~HostEnvironmentMapController() {
    cudaFree(d_map);
    delete[] h_gridSnapshot;
}

void HostEnvironmentMapController::runIteration(float a, float b, float c, float d) {
    // Launch GPU kernel
    dim3 threads(16, 16);
    dim3 blocks((width + threads.x - 1)/threads.x, (height + threads.y - 1)/threads.y);
    
    iterateKernel<<<blocks, threads>>>(d_map, a, b, c, d);
    cudaDeviceSynchronize(); // Wait for GPU to finish

    // Optional: Copy grid data back to host
    cudaMemcpy(h_gridSnapshot, d_map->grid, width*height*sizeof(float), cudaMemcpyDeviceToHost);
}