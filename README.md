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
- **A\***: GPU-accelerated A* path planning (see `environment_astar.cu`).

## Directory Structure

```
ControlSystem/
├── .gitignore
├── Control
│   ├── build.py
│   ├── codes.py
│   ├── comm.py
│   ├── include
│   │   ├── communication_system.h
│   │   ├── control_system.h
│   │   ├── environment.h
│   │   ├── environment_system.h
│   │   ├── main_system.h
│   │   ├── mission_system.h
│   │   ├── motion_system.h
│   │   ├── nlmpc.h
│   │   ├── subsystem.h
│   │   ├── topics.hpp
│   │   └── vehicle_model.h
│   └── src
│       ├── control_system.cpp
│       ├── environment_astar.cu
│       ├── environment_global.cu
│       ├── environment_helper.cpp
│       ├── environment_map.cu
│       ├── environment_system.cpp
│       ├── main.cpp
│       ├── main_system.cpp
│       ├── mission_system.cpp
│       ├── motion_system.cpp
│       ├── nlmpc.cpp
│       └── vehicle_model.cpp
├── Simulation
│   ├── Assets
│   │   ├── Models
│   │   │   ├── Environment
│   │   │   ├── Materials
│   │   │   ├── Missions
│   │   │   └── Vehicle
│   │   └── Scripts
│   │       ├── CameraPoser.cs
│   │       ├── EnvironmentTcpSubscriber.cs
│   │       ├── Missions
│   │       │   └── AsteroidFieldGenerator.cs
│   │       ├── SonarRayCast.cs
│   │       ├── SonarTCPPublisher.cs
│   │       ├── bridge.py
│   │       └── test.py
│   ├── Packages
│   └── ProjectSettings
├── Test
│   ├── Model
│   ├── il
│   ├── il-map
│   ├── map-entegrated-mpc
│   ├── mpc
│   └── ppo
└── environment.yml
└── README.md
└── TODO.md
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
    cd ControlSystem/Control
    ```
3. Build using the provided [build.py](build.py) script or with CMake:
    ```sh
    python build.py
    ```

## Troubleshooting

- **CUDA errors**: Ensure your GPU and driver are compatible with your CUDA toolkit version.
- **CasADi errors**: Make sure CasADi is installed and linked correctly.
- **See [NVIDIA CUDA GPUs](https://developer.nvidia.com/cuda-gpus) for compatibility.**

## License
