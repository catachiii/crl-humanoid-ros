# CRL Humanoid ROS

Code for humanoid robot simulation and control.

## Dependencies

### Core Dependencies

- [ROS2](https://docs.ros.org/) Humble
    - See [this](https://docs.ros.org/en/humble/Installation.html) for install ROS2 on your system.
- [colcon](https://colcon.readthedocs.io/en/released/index.html)
    - For using colcon to build ROS2 packages.
    - If you have installed ROS2 already, do:
    ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```
- [MuJoCo](https://github.com/google-deepmind/mujoco) 3.3.5
    - Physics simulation engine
- [Boost](https://www.boost.org/)
    - C++ libraries
- [Eigen](https://eigen.tuxfamily.org/) 3.4.0
    - Linear algebra library
- [cxxopts](https://github.com/jarro2783/cxxopts)
    - For parsing command line arguments
- [nlohmann-json](https://github.com/nlohmann/json) 3.10.0
    - JSON library for C++
- OpenGL and GLFW3
    - For graphics and visualization

### ROS2 Additional Packages

```bash
sudo apt install ros-humble-rmw-cyclonedx-cpp
sudo apt install ros-humble-rosidl-generator-dds-idl
sudo apt install ros-humble-geometry-msgs
sudo apt install ros-humble-sensor-msgs
sudo apt install ros-humble-std-msgs
sudo apt install ros-humble-std-srvs
```

### Installing Dependencies Manually

If not using the dev container, install the dependencies:

```bash
sudo apt update && sudo apt install -y
    build-essential
    cmake
    pkg-config
    libboost-all-dev
    libeigen3-dev
    nlohmann-json3-dev
    libglm-dev
    libcxxopts-dev
    libglfw3-dev
    freeglut3-dev
    mesa-utils
    libgl1-mesa-glx
    libgl1-mesa-dri
```

#### Installing MuJoCo 3.3.5

```bash
# Download and install MuJoCo 3.3.5
cd /tmp
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz
sudo mv mujoco-3.3.5 /opt/mujoco

# Create symlinks for system library
sudo ln -s /opt/mujoco/lib/libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so.3.3.5
sudo ln -s libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so

# Create pkg-config file for MuJoCo
sudo mkdir -p /usr/local/lib/pkgconfig
sudo tee /usr/local/lib/pkgconfig/mujoco.pc > /dev/null <<EOF
prefix=/opt/mujoco
exec_prefix=\${prefix}
libdir=\${exec_prefix}/lib
includedir=\${prefix}/include

Name: mujoco
Description: MuJoCo Physics Simulator
Version: 3.3.5
Libs: -L\${libdir} -lmujoco
Cflags: -I\${includedir}
EOF

# Update library cache
sudo ldconfig
```

## Subpackages

- `crl_ros_helper`: helper functions for ROS integration
- `crl_fsm`: finite state machine implementation
- `crl_fsm_msgs`: message definitions for crl_fsm
- `crl_humanoid_msgs`: message definitions for crl_humanoid packages
- `crl_humanoid_commons`: common interfaces and utilities
- `crl_humanoid_monitor`: GUI application for robot operations
- `crl_humanoid_simulator`: simulation tool for software-in-the-loop testing with MuJoCo
- `unitree_ros2`
  - `unitree_api`: Unitree API interfaces
  - `unitree_go`: Unitree Go series robot support

## Getting Started

Note: If you use conda or virtualenv, first deactivate the python environment. It's recommended to use system python for ROS2.

### Using Dev Container (Recommended)

1. Open this repository in VS Code with the Dev Containers extension
2. Reopen in container when prompted
3. All dependencies will be automatically installed

### Building the Workspace

1. Set up your environment:
    ```bash
    # Replace ".bash" with your shell if you're not using bash
    # Possible values are: setup.bash, setup.sh, setup.zsh
    source /opt/ros/humble/setup.bash
    ```

2. Build packages with colcon:
    ```bash
    # Build all packages
    ./build.sh

    # Or build specific packages
    colcon build --packages-up-to crl_humanoid_simulator --cmake-args -DCMAKE_BUILD_TYPE=Release
    ```

3. Source the environment:
    ```bash
    # source setup.zsh if you use zsh
    source install/setup.bash
    ```

### Running the Simulation

1. Start the G1 humanoid simulation:
    ```bash
    ros2 run crl_humanoid_simulator sim
    ```

2. In another terminal, run the monitor application:
    ```bash
    # Maps 'monitor_joystick' topic to 'remote' topic for joystick control
    ros2 run crl_humanoid_monitor monitor --ros-args -r monitor_joystick:=remote
    ```

3. Alternatively, use the launch file to run both simulator and monitor:
    ```bash
    ros2 launch crl_humanoid_simulator g1.py
    ```
    - The launch file runs monitor and simulator with the namespace `g1_sim`
    - It loads pre-defined parameters from the config directory

## Development

### Available Tasks

You can use the following VS Code tasks or run them directly:

- `build`: Build the workspace (default)
- `debug`: Build with debug symbols
- `test`: Run all unit tests
- `clean`: Clean build artifacts
- `fix`: Reformat code with uncrustify
- `setup`: Set up the workspace dependencies

### Building Components

- **Simulator only**: `colcon build --packages-up-to crl_humanoid_simulator`
- **Monitor only**: `colcon build --packages-up-to crl_humanoid_monitor`
- **All packages**: `./build.sh`

## Contributing

Please follow the coding standards:
- Use `ament_uncrustify` for code formatting
- Run `ament_cpplint` for C++ linting
- Run `flake8` and `pep257` for Python code
