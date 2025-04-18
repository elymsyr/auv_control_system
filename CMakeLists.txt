cmake_minimum_required(VERSION 3.12)
project(control_system LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_path(CASADI_INCLUDE_DIR casadi/casadi.hpp HINTS $ENV{CASADI_PREFIX}/include)
find_library(CASADI_LIBRARY NAMES casadi HINTS ${CASADI_INCLUDE_DIR}/../lib $ENV{CASADI_PREFIX}/lib)
set(CASADI_LIBRARIES ${CASADI_LIBRARY})

# Find dependencies
find_package(CASADI REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
find_package(OpenSSL REQUIRED)
find_package(ZLIB REQUIRED)

# For CasADi versions that don't export namespace targets
include_directories(control_system PRIVATE
    include
    ${CASADI_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${OPENSSL_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIRS}
)

add_executable(control_system
    src/main.cpp
    src/sensor_system.cpp
    src/control_system.cpp
    src/communication_system.cpp
    src/main_system.cpp
    src/environment_state.cpp
    src/environment_system.cpp
    src/motion_system.cpp
    src/system_data.cpp
)

target_link_libraries(control_system PRIVATE
    ${CASADI_LIBRARIES}
    Boost::system
    OpenSSL::SSL
    OpenSSL::Crypto
    ZLIB::ZLIB
    pthread
    dl  # CasADi often requires -ldl
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
message(STATUS "CasADi version: ${CASADI_VERSION}")
message(STATUS "Boost version: ${Boost_VERSION}")
message(STATUS "OpenSSL version: ${OPENSSL_VERSION}")
message(STATUS "CASADI_INCLUDE_DIR: ${CASADI_INCLUDE_DIR}")
message(STATUS "Boost_INCLUDE_DIRS: ${Boost_INCLUDE_DIRS}")
message(STATUS "OPENSSL_INCLUDE_DIR: ${OPENSSL_INCLUDE_DIR}")
message(STATUS "ZLIB_INCLUDE_DIRS: ${ZLIB_INCLUDE_DIRS}")

# Install target (optional)
install(TARGETS control_system
    RUNTIME DESTINATION bin
)