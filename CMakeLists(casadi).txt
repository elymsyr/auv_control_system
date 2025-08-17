cmake_minimum_required(VERSION 3.18)
project(control_system LANGUAGES CXX CUDA)

if(DEFINED ENV{CONDA_PREFIX})
  list(APPEND CMAKE_PREFIX_PATH $ENV{CONDA_PREFIX})
endif()

# ─── Language Standards ─────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD        17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS      OFF)

set(CMAKE_CUDA_STANDARD        14)
set(CMAKE_CUDA_STANDARD_REQUIRED ON)

# ─── CasADi Configuration ───────────────────────────────────────────────────────
set(CASADI_LINK_LIBS casadi)

# ─── Dependencies ───────────────────────────────────────────────────────────────
find_package(CUDAToolkit REQUIRED)
find_package(CASADI REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED COMPONENTS dnn)
find_package(ZLIB     REQUIRED)

# ─── Include Directories ───────────────────────────────────────────────────────
file(GLOB_RECURSE SOURCES 
    "src/*.cpp"
    "src/*.cu"
    "src/*/*.cpp"
    "src/*/*.cu"
)

include_directories(
    include
    ${CASADI_INCLUDE_DIR}
    ${ZeroMQ_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${ZLIB_INCLUDE_DIRS}
)

# ─── Executable & Sources ───────────────────────────────────────────────────────
add_executable(control_system
    src/main.cpp
    src/system/main_system.cpp
    src/system/control_system.cpp
    src/system/environment_system.cpp
    src/system/motion_system.cpp
    src/system/mission_system.cpp
    src/mapping/environment_astar.cu
    src/mapping/environment_global.cu
    src/mapping/environment_helper.cpp
    src/mapping/environment_map.cu
    src/control/nlmpc.cpp
    src/control/vehicle_model.cpp
    src/mission/mission_sonar_imp.cpp
)

# Include directories
target_include_directories(control_system PRIVATE
  ${CMAKE_SOURCE_DIR}/include
  ${CUDAToolkit_INCLUDE_DIRS}
)

# Link Conda's library directory
if(DEFINED ENV{CONDA_PREFIX})
  target_link_directories(control_system PRIVATE $ENV{CONDA_PREFIX}/lib)
endif()

# ─── CUDA-specific compile options ─────────────────────────────────────────────
# Enable separable compilation (useful for larger CUDA projects)
set_target_properties(control_system PROPERTIES CUDA_SEPARABLE_COMPILATION ON)

# ─── Link Libraries ────────────────────────────────────────────────────────────
target_link_libraries(control_system PRIVATE
    ${CASADI_LINK_LIBS} 
    ${ZeroMQ_LIBRARY}
    Boost::system
    zmq
    ${OpenCV_LIBS}
    ZLIB::ZLIB
    pthread
    dl
)

# ─── Optional: Arch-specific flags ─────────────────────────────────────────────
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    target_compile_options(control_system PRIVATE 
        -march=armv8-a
        -mtune=cortex-a57
        -O3
    )
    target_link_options(control_system PRIVATE
        -Wl,--as-needed
    )
endif()

# ─── Install ───────────────────────────────────────────────────────────────────
install(TARGETS control_system
    RUNTIME DESTINATION bin
)

# ─── Summary ───────────────────────────────────────────────────────────────────
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "C++ standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "CUDA standard: ${CMAKE_CUDA_STANDARD}")
message(STATUS "CUDA host compiler: ${CMAKE_CUDA_HOST_COMPILER}")
message(STATUS "CasADi include path: ${CASADI_INCLUDE_DIR}")
message(STATUS "Conda prefix: $ENV{CONDA_PREFIX}")