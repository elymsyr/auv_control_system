# ControlSystem

A modular, GPU-accelerated control system for Autonomous Underwater Vehicles (AUVs). The system is fully customizable and designed to operate in mission- and environment-agnostic scenarios.

This repository contains only the latest version of the main control system. For other AUV components, see:

- [**Simulation Systems**](https://github.com/elymsyr/auv_simulation)
- [**Vision Systems**](https://github.com/elymsyr/auv_vision)
- [**Control Models**](https://github.com/elymsyr/auv_control_model)
- [**LLM Agent**](https://github.com/elymsyr/auv_llm_agent)
- [**User Interface**](https://github.com/Sannora/auv_User-Interface)

---

## Docs

- [Modular System](docs/MODULAR_README.md)
- [Vehicle Dynamics](docs/DYNAMICS_README.md)

---

## Features

- **Non-Linear Model Predictive Control (NL-MPC)** using CasADi for advanced trajectory tracking and control.
- **Imitation learning**: Distill CasADi-based nonlinear MPC into neural network controllers for faster iteration.
- **CUDA-accelerated environment mapping** for real-time collision checking and safe navigation.
- **GPU-accelerated A\*** path planning for efficient route computation.
- **Fossen equations** for accurate marine vehicle dynamics modeling.
- **Flexible vehicle model configuration** via JSON.
- **Modular architecture** with custom templated components for rapid development.
- **Pub/Sub communication**: ROS2-like communication using ZeroMQ.
- **LLM Agent**: Integrate a Large Language Model agent for high-level mission planning and autonomous command interpretation.

---

## Requirements

- **CUDA Toolkit** (11.x or 12.x recommended, developed on 12.4.1)
- **NVIDIA GPU** with compatible driver
- **Conda**
- **C++17** or newer
- **CMake**

---

## Building

### Docker 

Recommended when the original GUI is used on local web.

1. Pull and run container:
    ```sh
    docker pull elymsyr/control_system:latest
    docker run --gpus all -it elymsyr/control_system -h
    ```

2. Clone this repository:
    ```sh
    git clone https://github.com/elymsyr/auv_control_system.git
    cd auv_control_system
    ```

3. Use test UI to control the system:
    ```sh
    pip install zmq
    python connection/comm.py
    ```

### CMake 

Recommended when the test UI is used on local.

1. Install dependencies (CUDA, CasADi, nlohmann/json).
2. Clone this repository:
    ```sh
    git clone https://github.com/elymsyr/auv_control_system.git
    cd auv_control_system
    ```
3. Build with CMake:
    ```sh
    conda env create -f environment.yml
    conda run -n mp_test bash -c " \
      cd /app && \
      rm -rf build && \
      mkdir build && \
      cd build && \
      cmake -DBOOST_ROOT=\$CONDA_PREFIX \
        -DCMAKE_CXX_COMPILER=\$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++ \
        -DCMAKE_CUDA_HOST_COMPILER=\$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-gcc \
        -DCMAKE_PREFIX_PATH=\$CONDA_PREFIX \
        -DCMAKE_BUILD_TYPE=Debug .. && \
      make -j\$(nproc) \
      "
    ```

4. See help and run:
    ```sh
    ./control_system -h
    ```

5. Use test UI to control the system:
    ```sh
    pip install zmq
    conda run -n mp_test bash -c "python ../connection/comm.py"
    ```

---

## License
[GNU GENERAL PUBLIC LICENSE](LICENSE)

## Development Process

[TR] Ocak 2025 - Mayıs 2025 geliştirme sürecine bakmak için [Notion](https://peridot-slash-ceb.notion.site/Sualt-Arac-Yaz-l-m-Tak-m-1d34a7fa163f8126b44fc97fc5dc5710) sayfasını ziyaret edebilirsiniz.

## Security Notice

**This project is under active development and is not production-ready.**  
There may be untested features, incomplete security measures, or vulnerabilities.  
**It is not recommended to use this system in safety-critical or production environments.**
