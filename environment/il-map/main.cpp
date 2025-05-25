// main.cpp
#include "HostController.hpp"

int main() {
    // CPU-side initialization
    HostEnvironmentMapController controller(128, 128);
    
    // Main loop (CPU-driven)
    for (int iter = 0; iter < 1000; ++iter) {
        // Control parameters from CPU
        float a = computeA(); // Your CPU logic
        float b = computeB();
        
        // Trigger GPU iteration
        controller.runIteration(a, b, 0.0f, 0.1f);
    }
    
    return 0;
}
