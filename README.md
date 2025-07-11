# ControlSystem

A modular, GPU-accelerated control system for Autonomous Underwater Vehicles (AUVs). The system is implemented fully customizable and to work under mission and environment-free situations

This project includes only the applied last version of the main control system. See other projects to reach all the AUV components:

- [**Simulation Systems**](https://github.com/elymsyr/auv_simulation)
- [**Vision Systems**](https://github.com/mehmetcullu/au_vision)
- **GUI Systems:** *will be added*

## Features

- **Non-Linear Model Predictive Control (NL-MPC)** using CasADi for advanced trajectory tracking and control.
- **Imitation learning**: Distill CasADi-based nonlinear MPC into neural network controllers for faster iteration.
- **CUDA-accelerated environment mapping** for real-time collision checking and safe navigation.
- **GPU-accelerated A\*** path planning for efficient route computation.
- **Fossen equations** for accurate marine vehicle dynamics modeling.
- **Flexible vehicle model configuration** via JSON.
- **Modular architecture** with custom templated components for rapid development
- **Pub/Sub communication** ROS2 like communication

## Modular Structure

The system follows a component-based architecture where modules communicate through ZeroMQ publisher/subscriber patterns. Key components:

### Core Systems
- **`main_system.h`**: Orchestrates system initialization and module coordination
- **`subsystem.h`**: Base template for all modules (enables standardized communication)
- **`communication_methods.h`**: Implements ZeroMQ-based pub/sub communication
- **`topics.hpp`**: Defines standardized message formats (ROS-like)

### Communication Architecture
- **ZeroMQ Implementation**:
  - TCP-based PUB/SUB pattern for inter-process communication
  - Protobuf-serialized messages for efficient data transfer
  - Configurable endpoints (e.g., `tcp://localhost:5555`)
- **Message Handling**: Fast and customizable structure via templated pub/sub methods

### Functional Modules
- **Control System (`control_system.h`)**
  - Interfaces with NLMPC controller (`nlmpc.h`)
  - Implements vehicle dynamics models (`vehicle_model.h`)
- **Environment System (`environment_system.h`)**
  - Updates vehicle state
- **Mission System (`mission_system.h`)**
  - Executes mission profiles (`mission.h`)
  - Supports specialized missions (e.g. `mission_sonar_imp.h`)
- **Motion System (`motion_system.h`)**
  - Integrates path planning with control outputs
  - Manages actuator interfaces

### Key Advantages
1. **Realistic Vehicle Dynamics:** 6-DOF Fossen Equations implementation
2. **Neural Network Implemented NL-MPC:** Faster iterations
1. **Hot-swappable modules**: Components can be replaced at runtime
2. **Template-based development**: 
   ```cpp
   // Example module declaration
   class MissionSystem : public SubSystem<SonarTopic, ControlTopic> {...};
   ```
3. **Cross-component compatibility:** Standardized topics ensure interoperability
4. **GPU-CPU hybrid processing:** Critical paths optimized with CUDA kernels


## Requirements

- **CUDA Toolkit** (11.x or 12.x recommended)
- **NVIDIA GPU** with compatible driver
- **Conda**
- **C++17** or newer
- **CMake** (recommended for building)

## Building

1. Install dependencies (CUDA, CasADi, nlohmann/json).
2. Clone this repository:
    ```sh
    git clone https://github.com/yourusername/auv_control_system.git
    cd auv_control_system/Control
    ```
3. Build using the provided [build.py](build.py) script or with CMake:
    ```sh
    python build.py
    ```

## Troubleshooting

- **CUDA errors**: Ensure your GPU and driver are compatible with your CUDA toolkit version.
- **See [NVIDIA CUDA GPUs](https://developer.nvidia.com/cuda-gpus) for compatibility.**

## License
[GNU GENERAL PUBLIC LICENSE](LICENSE)

## Security Notice

**This project is under active development and is not production-ready.**  
There may be untested features, incomplete security measures, or vulnerabilities.  
**It is not recommended to use this system in safety-critical or production environments.**
