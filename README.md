# CRL Humanoid ROS

![ROS Demo](miscs/videos/ros_demo.gif)

A ROS 2 framework for humanoid robot simulation and control, developed by the [Computational Robotics Lab (CRL)](https://crl.ethz.ch) at ETH Zurich. Supports the **Unitree G1** and **LimX Tron1A** platforms.

## Overview

This repository provides a modular ROS 2 workspace for:

- **Simulation** — MuJoCo-based software-in-the-loop testing
- **Hardware control** — Real-time communication with robot hardware
- **Monitoring** — GUI for visualization and operator control
- **Locomotion controllers** — RL-based, motion-imitation, and goal-directed controllers

## Repository Structure

```
crl-humanoid-ros/
├── crl_ros_helper/               # ROS 2 helper utilities
├── crl_fsm/                      # Finite state machine framework
├── crl_fsm_msgs/                 # FSM message/service definitions
│
├── crl_humanoid_msgs/            # Humanoid message definitions
├── crl_humanoid_commons/         # Common interfaces and utilities
├── crl_humanoid_monitor/         # MuJoCo-based GUI monitor
├── crl_humanoid_simulator/       # MuJoCo physics simulator
├── crl_humanoid_hardware/        # Hardware interface (Unitree SDK2 / LimX SDK)
│
├── crl_g1_rlcontroller/          # G1 RL locomotion controller
├── crl_g1_mimiccontroller/       # G1 DeepMimic / DeepTrack controllers
├── crl_g1_goalcontroller_py/     # G1 goal-directed controller
│   ├── crl_g1_goalcontroller/    #   C++ launch/config layer
│   └── crl_g1_goalcontroller_python/  #   Python controller nodes
│
├── crl_tron1a_rlcontroller/      # Tron1A RL locomotion controller
│
├── crl_optitrack_ros/            # OptiTrack motion capture integration
│   ├── optitrack_msgs/           #   MoCap message definitions
│   ├── optitrack_adaptor/        #   ROS 2 adapter for OptiTrack
│   └── latency_checker/          #   MoCap latency measurement tool
│
├── limxsdk-lowlevel/             # LimX Dynamics low-level SDK
└── miscs/                        # Development environment scripts
```

### Package Dependency Graph

```
               ┌──────────────────────────────────────────────────┐
               │              Controller Packages                 │
               │  crl_g1_rlcontroller   crl_g1_mimiccontroller    │
               │  crl_g1_goalcontroller crl_tron1a_rlcontroller   │
               └──────────────────┬───────────────────────────────┘
                                  │
               ┌──────────────────▼───────────────────────────────┐
               │            Platform Packages                     │
               │  crl_humanoid_simulator  crl_humanoid_hardware   │
               │  crl_humanoid_monitor                            │
               └──────────────────┬───────────────────────────────┘
                                  │
               ┌──────────────────▼───────────────────────────────┐
               │             Core Packages                        │
               │  crl_humanoid_commons  crl_fsm  crl_ros_helper   │
               └──────────────────┬───────────────────────────────┘
                                  │
               ┌──────────────────▼───────────────────────────────┐
               │           Message Packages                       │
               │  crl_humanoid_msgs  crl_fsm_msgs  optitrack_msgs│
               └──────────────────────────────────────────────────┘
```

## Prerequisites

| Dependency | Version | Purpose |
|---|---|---|
| [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) | Humble Hawksbill | Middleware framework |
| [MuJoCo](https://github.com/google-deepmind/mujoco) | 3.3.5 | Physics simulation & GUI |
| [ONNX Runtime](https://onnxruntime.ai/) | 1.15.0 | Neural network inference (RL controllers) |
| [Eigen](https://eigen.tuxfamily.org/) | 3.4+ | Linear algebra |
| [Boost](https://www.boost.org/) | — | C++ utilities |
| [cxxopts](https://github.com/jarro2783/cxxopts) | 3.0+ | Command-line argument parsing |
| [nlohmann-json](https://github.com/nlohmann/json) | 3.10+ | JSON parsing |
| OpenGL / GLFW3 | — | Graphics (monitor GUI) |

## Installation

### 1. System Libraries

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake pkg-config \
  libboost-all-dev libeigen3-dev nlohmann-json3-dev libcxxopts-dev \
  libglfw3-dev freeglut3-dev mesa-utils libgl1-mesa-glx libgl1-mesa-dri
```

### 2. ROS 2 Packages

```bash
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime \
  ros-humble-ament-lint-auto \
  ros-humble-ament-lint-common

# For Unitree G1 hardware (CycloneDDS middleware)
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl
```

### 3. MuJoCo 3.3.5

```bash
cd /tmp
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz
sudo mv mujoco-3.3.5 /opt/mujoco

sudo ln -s /opt/mujoco/lib/libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so.3.3.5
sudo ln -s /usr/local/lib/libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so
sudo ldconfig
```

### 4. ONNX Runtime 1.15.0

Required by the RL and mimic controller packages.

```bash
mkdir /tmp/onnxInstall && cd /tmp/onnxInstall
wget -O onnx_archive.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.15.0
unzip onnx_archive.nupkg

# Choose the correct platform binary
sudo cp runtimes/linux-x64/native/libonnxruntime.so /usr/local/lib/libonnxruntime.so.1.15.0
sudo cp -r build/native/include/ /usr/local/include/onnxruntime/

cd /usr/local/lib
sudo ln -s libonnxruntime.so.1.15.0 libonnxruntime.so
sudo ldconfig
```

Then create the CMake config files so `find_package(onnxruntime)` works:

```bash
sudo mkdir -p /usr/local/share/cmake/onnxruntime
```

Create `/usr/local/share/cmake/onnxruntime/onnxruntimeConfig.cmake`:

```cmake
include(FindPackageHandleStandardArgs)

get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include)
set(onnxruntime_LIBRARIES onnxruntime)
set(onnxruntime_CXX_FLAGS "")

find_library(onnxruntime_LIBRARY onnxruntime
    PATHS "${onnxruntime_INSTALL_PREFIX}/lib"
)

add_library(onnxruntime SHARED IMPORTED)
set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")

find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)
```

Create `/usr/local/share/cmake/onnxruntime/onnxruntimeConfigVersion.cmake`:

```cmake
set(PACKAGE_VERSION "1.15.0")

if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
    set(PACKAGE_VERSION_COMPATIBLE FALSE)
else()
    set(PACKAGE_VERSION_COMPATIBLE TRUE)
    if("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
        set(PACKAGE_VERSION_EXACT TRUE)
    endif()
endif()
```

### 5. Python Dependencies (for Goal Controller)

```bash
pip3 install numpy onnxruntime
```

## Building

This repository follows the standard ROS 2 workspace layout. Clone it into the `src/` directory of a colcon workspace:

```
ws/                          # workspace root — build from here
├── build/
├── install/
├── log/
└── src/
    └── crl-humanoid-ros/    # this repository
        ├── crl_ros_helper/
        ├── crl_fsm/
        ├── crl_fsm_msgs/
        ├── crl_humanoid_msgs/
        ├── crl_humanoid_commons/
        ├── crl_humanoid_monitor/
        ├── crl_humanoid_simulator/
        ├── crl_humanoid_hardware/
        ├── crl_g1_rlcontroller/
        ├── crl_g1_mimiccontroller/
        ├── crl_g1_goalcontroller_py/
        ├── crl_tron1a_rlcontroller/
        ├── crl_optitrack_ros/
        ├── limxsdk-lowlevel/
        └── miscs/
```

**All `colcon` commands must be run from the workspace root (`ws/`).**

```bash
# Setup
mkdir -p ws/src && cd ws/src
git clone <repo-url> crl-humanoid-ros
cd ../  # back to ws/

# Source ROS 2 and build
source /opt/ros/humble/setup.bash
colcon build --cmake-args "-DCMAKE_BUILD_TYPE=Release" --symlink-install

# Source the workspace overlay
source install/setup.bash
```

To build a single package with its dependencies:

```bash
colcon build --packages-up-to <package_name> --cmake-args "-DCMAKE_BUILD_TYPE=Release" --symlink-install
```

## Usage

### Monitor GUI

```bash
ros2 run crl_humanoid_monitor monitor
```

### Unitree G1

#### Simulation

```bash
# Simulator only
ros2 launch crl_humanoid_simulator g1.py

# RL controller (two policy variants)
ros2 launch crl_g1_rlcontroller g1_sim.py
ros2 launch crl_g1_rlcontroller g1_sim_v2.py

# DeepMimic / DeepTrack controllers
ros2 launch crl_g1_mimiccontroller g1_sim_deepmimic.py
ros2 launch crl_g1_mimiccontroller g1_sim_deeptrack.py

# Goal-directed controller
ros2 launch crl_g1_goalcontroller g1_sim.py
```

#### Hardware

```bash
# Hardware interface
ros2 launch crl_humanoid_hardware g1.py

# RL controller
ros2 launch crl_g1_rlcontroller g1.py
ros2 launch crl_g1_rlcontroller g1_v2.py

# DeepMimic / DeepTrack controllers
ros2 launch crl_g1_mimiccontroller g1_deepmimic.py
ros2 launch crl_g1_mimiccontroller g1_deeptrack.py

# Goal-directed controller
ros2 launch crl_g1_goalcontroller g1.py
```

### LimX Tron1A

#### Simulation

```bash
# Simulator only
ros2 launch crl_humanoid_simulator wf_tron1a.py

# RL controller
ros2 launch crl_tron1a_rlcontroller wf_tron1a_sim.py
```

#### Hardware

```bash
# Hardware interface
ros2 launch crl_humanoid_hardware wf_tron1a.py

# RL controller
ros2 launch crl_tron1a_rlcontroller wf_tron1a.py
```

## Package Details

### Core Infrastructure

| Package | Description |
|---|---|
| `crl_ros_helper` | ROS 2 helper classes and utilities for node management |
| `crl_fsm` | Compile-time finite state machine for orchestrating controller state transitions (see [crl_fsm/README.md](crl_fsm/README.md)) |
| `crl_fsm_msgs` | Service and message definitions (`StateSwitch.srv`, `StateInfo.msg`) used by the FSM |

### Humanoid Platform

| Package | Description |
|---|---|
| `crl_humanoid_msgs` | ROS 2 message types for sensor data, control commands, and monitoring |
| `crl_humanoid_commons` | Shared robot models, interfaces, and utility code used across all controllers |
| `crl_humanoid_monitor` | MuJoCo-based GUI for real-time visualization and operator interaction |
| `crl_humanoid_simulator` | MuJoCo physics simulator for software-in-the-loop testing |
| `crl_humanoid_hardware` | Hardware interface layer supporting Unitree SDK2 and LimX SDK |

### Controllers

| Package | Description |
|---|---|
| `crl_g1_rlcontroller` | Reinforcement-learning locomotion controller for Unitree G1 |
| `crl_g1_mimiccontroller` | DeepMimic and DeepTrack motion-imitation controllers for G1 |
| `crl_g1_goalcontroller` | Goal-directed locomotion controller for G1 (C++ + Python) |
| `crl_tron1a_rlcontroller` | Reinforcement-learning locomotion controller for LimX Tron1A |

### External & Auxiliary

| Package | Description |
|---|---|
| `limxsdk-lowlevel` | Low-level SDK for LimX Dynamics hardware |
| `optitrack_msgs` | Message definitions for OptiTrack motion capture data |
| `optitrack_adaptor` | ROS 2 adapter for the OptiTrack system (see [crl_optitrack_ros/README.md](crl_optitrack_ros/README.md)) |
| `latency_checker` | Tool for measuring motion capture system latency |

## License

All CRL packages are licensed under **ETH Zurich Computational Robotics Lab**. Third-party packages (`limxsdk-lowlevel`) retain their original licenses.

