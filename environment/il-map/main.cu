#include <iostream>
#include "EnvironmentMap.cuh"
#include <fstream>
#include <thrust/host_vector.h>  // Add this
#include <thrust/device_vector.h>
#include <vector_types.h>        // For int2

#define CHECK_CUDA_ERROR(call)                                                 \
do {                                                                           \
    cudaError_t err = call;                                                    \
    if (err != cudaSuccess) {                                                  \
        std::cerr << "CUDA error at " << __FILE__ << ":" << __LINE__ << ": "   \
                  << cudaGetErrorString(err) << std::endl;                     \
        exit(EXIT_FAILURE);                                                    \
    }                                                                          \
} while(0)

int main() {
    const int W = 129, H = 129;

    // 1. Create device-side instance using unified memory
    EnvironmentMap* d_map;
    CHECK_CUDA_ERROR(cudaMallocManaged(&d_map, sizeof(EnvironmentMap)));
    d_map->initialize(W, H);

    // 2. Generate random points using Thrust
    const int num_points = 100;
    thrust::host_vector<int2> h_coords(num_points);
    thrust::host_vector<uint8_t> h_values(num_points);

    // Initialize host data
    for(int i = 0; i < num_points; ++i) {
        h_coords[i] = make_int2(rand() % W, rand() % H);
        h_values[i] = rand() % 256;
    }

    // Copy to device
    thrust::device_vector<int2> d_coords = h_coords;
    thrust::device_vector<uint8_t> d_values = h_values;

    // 3. Create batch structure
    PointBatch batch{
        num_points,
        thrust::raw_pointer_cast(d_coords.data()),
        thrust::raw_pointer_cast(d_values.data())
    };

    // 4. Launch optimized kernel
    const int BLOCK_SIZE = 256;
    ultraFastUpdateKernel<<<(num_points + BLOCK_SIZE - 1)/BLOCK_SIZE, BLOCK_SIZE>>>(
        d_map, batch
    );
    CHECK_CUDA_ERROR(cudaDeviceSynchronize());

    // 5. Export data
    std::ofstream rawFile("grid_data.bin", std::ios::binary);
    rawFile.write(reinterpret_cast<char*>(d_map->grid), W * H);
    rawFile.close();

    // 4. Run iterations
    for (int i = 0; i < 30; ++i) {
        map->iterate(12.2f, 10.4f);
        CHECK_CUDA_ERROR(cudaGetLastError());
        CHECK_CUDA_ERROR(cudaDeviceSynchronize());
    }

    std::ofstream rawFile_("grid_data1.bin", std::ios::binary);
    rawFile_.write(reinterpret_cast<char*>(d_map->grid), W * H);
    rawFile_.close();

    // 6. Cleanup
    d_map->cleanup();
    CHECK_CUDA_ERROR(cudaFree(d_map));
    
    return 0;
}
