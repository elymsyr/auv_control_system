cmake_minimum_required(VERSION 3.12)
project(control_system LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Suppress Boost policy warning
cmake_policy(SET CMP0167 NEW)

# Find dependencies first
find_package(Boost 1.65 REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)

# Add executable target with source files
add_executable(control_system
    src/main.cpp
    src/mission_system.cpp
    src/sensor_system.cpp
    src/control_system.cpp
    src/communication_system.cpp
    src/main_system.cpp
    src/environment_state.cpp
    src/environment_system.cpp
    src/motion_system.cpp
    src/system_data.cpp
)

# Target-specific include directories
target_include_directories(control_system PRIVATE
    include
    ${Boost_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIRS}
)

# Target link libraries using modern CMake targets
target_link_libraries(control_system PRIVATE
    Boost::system
    OpenSSL::SSL
    OpenSSL::Crypto
    pthread
    ${ZLIB_LIBRARIES}
)

# Architecture-specific optimizations
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

# Optional: Print configuration summary
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
message(STATUS "Boost version: ${Boost_VERSION}")
message(STATUS "OpenSSL version: ${OPENSSL_VERSION}")

# Install target (optional)
install(TARGETS control_system
    RUNTIME DESTINATION bin
)