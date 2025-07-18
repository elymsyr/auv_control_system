#include <iostream>
#include "model_inference.h"
#include "utils.h"

int main() {
    // Initialize the model inference object
    ModelInference model;

    // Load the model onto the GPU
    if (!model.loadModel("path/to/model/file")) {
        std::cerr << "Failed to load model." << std::endl;
        return -1;
    }

    // Prepare input data
    float* inputData = prepareInputData("path/to/input/data");
    
    // Run inference
    float* outputData = model.runInference(inputData);
    
    // Process output data
    processOutputData(outputData);

    // Release resources
    model.releaseResources();
    
    // Clean up
    delete[] inputData;
    delete[] outputData;

    return 0;
}