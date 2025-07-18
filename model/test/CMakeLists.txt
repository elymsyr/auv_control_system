cmake_minimum_required(VERSION 3.10)

# Set the project name
project(cpp_gpu_model_app)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/model_inference.cpp
    src/utils.cpp
)

# Create the executable
add_executable(cpp_gpu_model_app ${SOURCES})

# Link any necessary libraries (e.g., CUDA, if applicable)
# find_package(CUDA REQUIRED)
# target_link_libraries(cpp_gpu_model_app ${CUDA_LIBRARIES})