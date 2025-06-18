# ControlSystem

A modular control system framework for robotics and autonomous vehicles, featuring:

- **Model Predictive Control (MPC)** using CasADi
- **CUDA-accelerated environment mapping and barrier functions**
- **A* path planning (GPU-accelerated)**
- Flexible vehicle model configuration via JSON

## Features

- **VehicleModel**: Loads vehicle parameters from JSON and provides symbolic dynamics for MPC.
- **NonlinearMPC**: Implements nonlinear MPC using CasADi, with warm-starting and robust fallback.
- **EnvironmentMap**: CUDA-accelerated occupancy grid for mapping and collision checking.
- **Barrier Functions**: Fast GPU-based evaluation for safe trajectory planning.
- **A\***: GPU-accelerated A* path planning (see `AStar.cu`).

## Directory Structure

```
ControlSystem/
├── Simulation
│   ├── Assets
│   │   ├── Models
│   │   │   ├── Environment
│   │   │   │   └── Pool.prefab
│   │   │   ├── Materials
│   │   │   │   ├── line.mat
│   │   │   │   ├── shapes.mat
│   │   │   │   ├── walls.mat
│   │   │   │   └── water.mat
│   │   │   ├── Missions
│   │   │   │   └── Line Mission.prefab
│   │   │   └── Vehicle
│   │   │       └── vehicle.fbx
│   │   └── Scripts
│   │       ├── CameraPoser.cs
│   │       ├── SonarRayCast.cs
│   │       ├── SonarTCPPublisher.cs
│   │       ├── SonarTCPSubscriber.cs
│   │       ├── bridge.py
│   │       └── test.py
│   ├── Control
│   │   ├── build.py
│   │   ├── codes.py
│   │   ├── comm.py
│   │   ├── include
│   │   │   ├── communication_system.h
│   │   │   ├── control_system.h
│   │   │   ├── environment.h
│   │   │   ├── environment_system.h
│   │   │   ├── main_system.h
│   │   │   ├── mission_system.h
│   │   │   ├── motion_system.h
│   │   │   ├── nlmpc.h
│   │   │   ├── subsystem.h
│   │   │   ├── topics.hpp
│   │   │   └── vehicle_model.h
│   │   └── src
│   │       ├── control_system.cpp
│   │       ├── environment_astar.cu
│   │       ├── environment_global.cu
│   │       ├── environment_helper.cpp
│   │       ├── environment_map.cu
│   │       ├── environment_system.cpp
│   │       ├── main.cpp
│   │       ├── main_system.cpp
│   │       ├── mission_system.cpp
│   │       ├── motion_system.cpp
│   │       ├── nlmpc.cpp
│   │       └── vehicle_model.cpp
│   ├── Packages
│   └── ProjectSettings
├── build.py
├── codes.py
├── comm.py
├── include
│   ├── communication_system.h
│   ├── control_system.h
│   ├── environment.h
│   ├── environment_system.h
│   ├── main_system.h
│   ├── mission_system.h
│   ├── motion_system.h
│   ├── nlmpc.h
│   ├── subsystem.h
│   ├── topics.hpp
│   └── vehicle_model.h
├── src
│   ├── control_system.cpp
│   ├── environment_astar.cu
│   ├── environment_global.cu
│   ├── environment_helper.cpp
│   ├── environment_map.cu
│   ├── environment_system.cpp
│   ├── main.cpp
│   ├── main_system.cpp
│   ├── mission_system.cpp
│   ├── motion_system.cpp
│   ├── nlmpc.cpp
│   └── vehicle_model.cpp
└── tree.py
```

## Requirements

- **CUDA Toolkit** (11.x or 12.x recommended)
- **NVIDIA GPU** with compatible driver
- **CasADi** (for MPC)
- **nlohmann/json** (for config parsing)
- **C++17** or newer
- **CMake** (recommended for building)

## Building

1. Install dependencies (CUDA, CasADi, nlohmann/json).
2. Clone this repository:
    ```sh
    git clone https://github.com/yourusername/ControlSystem.git
    cd ControlSystem
    ```
3. See [build](build.py) with CMake

## Usage

- **Configure your vehicle**: Edit the JSON config file with your vehicle’s parameters.
- **Run the MPC**: Use the `NonlinearMPC` class in your main application.
- **Map the environment**: Use `EnvironmentMap` and related CUDA kernels for fast mapping and collision checking.
- **Plan paths**: Use the GPU-accelerated A* implementation.

## Troubleshooting

- **CUDA errors**: Ensure your GPU and driver are compatible with your CUDA toolkit version.
- **CasADi errors**: Make sure CasADi is installed and linked correctly.
- **See [NVIDIA CUDA GPUs](https://developer.nvidia.com/cuda-gpus) for compatibility.**

## License

MIT License

---

*This project is under active development. Contributions and issues are welcome!*