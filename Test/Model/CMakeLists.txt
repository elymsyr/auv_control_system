cmake_minimum_required(VERSION 3.18)
project(test LANGUAGES CXX CUDA)

# Prioritize Conda environment
if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

# Language standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CUDA_STANDARD 14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# CasADi setup
set(CASADI_LINK_LIBS casadi)

# Dependencies (Conda versions will be found via CMAKE_PREFIX_PATH)
find_package(CUDAToolkit REQUIRED)
find_package(CASADI REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
find_package(ZLIB REQUIRED)
find_package(HDF5 REQUIRED COMPONENTS CXX)  # Provides HDF5::HDF5 target
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# Executable
add_executable(test
  src/environment_astar.cu
  src/environment_global.cu
  src/environment_map.cu
  src/environment_helper.cpp
  src/main.cpp
  src/nlmpc.cpp
  src/vehicle_model.cpp
)

# Include directories
target_include_directories(test PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CUDAToolkit_INCLUDE_DIRS}
)

# Link Conda's library directory
if(DEFINED ENV{CONDA_PREFIX})
  target_link_directories(test PRIVATE $ENV{CONDA_PREFIX}/lib)
endif()

# CUDA options
target_compile_options(test PRIVATE
  $<$<COMPILE_LANGUAGE:CUDA>:${CMAKE_CUDA_FLAGS}>
)
set_target_properties(test PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# Linking
target_link_libraries(test PRIVATE
  ${CASADI_LINK_LIBS}
  Boost::system
  ZLIB::ZLIB
  HDF5::HDF5  # Use imported target
  pthread
  dl
  OpenGL::GL
  glfw
)

# ─── Optional: Arch-specific flags ─────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(test PRIVATE 
        -march=armv8-a
        -mtune=cortex-a57
        -O3
    )
    target_link_options(test PRIVATE
        -Wl,--as-needed
    )
endif()

# ─── Install ───────────────────────────────────────────────────────────────────
install(TARGETS test
    RUNTIME DESTINATION bin
)

# ─── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA standard: ${CMAKE_CUDA_STANDARD}")
message(STATUS "CUDA host compiler: ${CMAKE_CUDA_HOST_COMPILER}")
message(STATUS "CasADi include path: ${CASADI_INCLUDE_DIR}")
message(STATUS "Conda prefix: $ENV{CONDA_PREFIX}")
