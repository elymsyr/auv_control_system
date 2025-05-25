// HostController.hpp
#include <cuda_runtime.h>

class HostEnvironmentMapController {
public:
    HostEnvironmentMapController(int width, int height);
    ~HostEnvironmentMapController();

    // Called from CPU each iteration
    void runIteration(float a, float b, float c, float d);

private:
    EnvironmentMap* d_map; // Device pointer to GPU class instance
    float* h_gridSnapshot; // Host-side copy of grid data (optional)
};