# crl-humanoid-ros

Code for CRL's humanoids (unitree G1, pnd adam, limx tron1)

## Dependencies

### Core System Dependencies

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
  - Physics simulation engine (required for monitor GUI and simulator)
- [Boost](https://www.boost.org/)
  - C++ libraries (dependency of unitree_legged_sdk)
- [Eigen](https://eigen.tuxfamily.org/) 3.4+
  - Linear algebra library
- [cxxopts](https://github.com/jarro2783/cxxopts) 3.0.0
  - For parsing command line arguments
- [nlohmann-json](https://github.com/nlohmann/json) 3.10+

### ROS2 Humble Dependencies

Most packages are included with ROS2 Humble desktop installation. Only install additional packages if needed:

```bash
# Additional ROS2 Humble packages (if not already installed)
sudo apt install \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common
```

### Hardware-Specific ROS2 Dependencies

Additional middleware packages for Unitree G1 hardware:

```bash
sudo apt install -y ros-humble-rmw-cyclonedds-cpp ros-humble-rosidl-generator-dds-idl
```

### ONNX Runtime (for RL controller)
- [onnxruntime](https://onnxruntime.ai/) 1.15.0
   - Install onnxruntime 1.15.0 as follows:
   ```bash
   mkdir /tmp/onnxInstall
   cd /tmp/onnxInstall
   wget -O onnx_archive.nupkg https://www.nuget.org/api/v2/package/Microsoft.ML.OnnxRuntime/1.15.0
   unzip onnx_archive.nupkg
   # IMPORTANT! Choose the correct platform.
   sudo cp runtimes/linux-x64/native/libonnxruntime.so /usr/local/lib/libonnxruntime.so.1.15.0
   sudo cp -r build/native/include/ /usr/local/include/onnxruntime/
   cd /usr/local/lib
   sudo ln -s libonnxruntime.so.1.15.0 libonnxruntime.so
   sudo ldconfig
   ```
   - Create ```onnxruntimeConfigVersion.cmake``` under the ```/usr/local/share/cmake/onnxruntime``` directory with the following content.
   ```cmake
   set(PACKAGE_VERSION "1.15.0")

   # Check whether the requested PACKAGE_FIND_VERSION is compatible
   if("${PACKAGE_VERSION}" VERSION_LESS "${PACKAGE_FIND_VERSION}")
      set(PACKAGE_VERSION_COMPATIBLE FALSE)
   else()
      set(PACKAGE_VERSION_COMPATIBLE TRUE)
      if("${PACKAGE_VERSION}" VERSION_EQUAL "${PACKAGE_FIND_VERSION}")
         set(PACKAGE_VERSION_EXACT TRUE)
      endif()
   endif()
   ```
   - Create ```onnxruntimeConfig.cmake``` under the ```/usr/local/share/cmake/onnxruntime``` directory with the following content.
   ```cmake
   # Custom cmake config file by jcarius to enable find_package(onnxruntime) without modifying LIBRARY_PATH and LD_LIBRARY_PATH
   #
   # This will define the following variables:
   #   onnxruntime_FOUND        -- True if the system has the onnxruntime library
   #   onnxruntime_INCLUDE_DIRS -- The include directories for onnxruntime
   #   onnxruntime_LIBRARIES    -- Libraries to link against
   #   onnxruntime_CXX_FLAGS    -- Additional (required) compiler flags

   include(FindPackageHandleStandardArgs)

   # Assume we are in <install-prefix>/share/cmake/onnxruntime/onnxruntimeConfig.cmake
   get_filename_component(CMAKE_CURRENT_LIST_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)
   get_filename_component(onnxruntime_INSTALL_PREFIX "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

   set(onnxruntime_INCLUDE_DIRS ${onnxruntime_INSTALL_PREFIX}/include)
   set(onnxruntime_LIBRARIES onnxruntime)
   set(onnxruntime_CXX_FLAGS "") # no flags needed


   find_library(onnxruntime_LIBRARY onnxruntime
      PATHS "${onnxruntime_INSTALL_PREFIX}/lib"
   )

   add_library(onnxruntime SHARED IMPORTED)
   set_property(TARGET onnxruntime PROPERTY IMPORTED_LOCATION "${onnxruntime_LIBRARY}")
   set_property(TARGET onnxruntime PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${onnxruntime_INCLUDE_DIRS}")
   set_property(TARGET onnxruntime PROPERTY INTERFACE_COMPILE_OPTIONS "${onnxruntime_CXX_FLAGS}")

   find_package_handle_standard_args(onnxruntime DEFAULT_MSG onnxruntime_LIBRARY onnxruntime_INCLUDE_DIRS)
   ```

### System Library Dependencies

Install the required system libraries:

```bash
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  pkg-config \
  libboost-all-dev \
  libeigen3-dev \
  nlohmann-json3-dev \
  libcxxopts-dev \
  libglfw3-dev \
  freeglut3-dev \
  mesa-utils \
  libgl1-mesa-glx \
  libgl1-mesa-dri
```

### Installing MuJoCo 3.3.5

MuJoCo is required for the monitor GUI and simulator packages:

```bash
# Download and install MuJoCo 3.3.5
cd /tmp
wget https://github.com/google-deepmind/mujoco/releases/download/3.3.5/mujoco-3.3.5-linux-x86_64.tar.gz
tar -xzf mujoco-3.3.5-linux-x86_64.tar.gz
sudo mv mujoco-3.3.5 /opt/mujoco

# Create symlinks for system library
sudo ln -s /opt/mujoco/lib/libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so.3.3.5
sudo ln -s libmujoco.so.3.3.5 /usr/local/lib/libmujoco.so

# Update library cache
sudo ldconfig
```

**Note:** You do not need to create a pkg-config file for MuJoCo. The build system uses hardcoded paths for MuJoCo and does not require pkg-config.

### Quick Installation Script

For convenience, you can install all dependencies with:

```bash
# Install ROS2 Humble packages
sudo apt update
sudo apt install -y \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-default-runtime \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    python3-colcon-common-extensions

# Install hardware-specific ROS2 packages (for Unitree G1)
sudo apt install -y \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosidl-generator-dds-idl

# Install system libraries
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    libboost-all-dev \
    libeigen3-dev \
    nlohmann-json3-dev \
    libcxxopts-dev \
    libglfw3-dev \
    freeglut3-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri
```

Then install MuJoCo and ONNX Runtime following the detailed instructions above.

### Python Environment Setup (for Goal Controller)

The `crl_g1_goalcontroller` package includes a Python-based controller that requires additional Python dependencies. It's recommended to use the system Python environment for simplicity.

Install required Python packages:

```bash
pip3 install numpy onnxruntime
```

**Note:** If you prefer to use a virtual environment or conda, ensure that the ROS 2 workspace can access the Python packages in that environment. Using system Python is simpler and avoids environment activation issues when launching ROS 2 nodes.

## Subpackages

### Core Infrastructure
- ```crl_ros_helper```: Helper functions for ROS integration
- ```crl_fsm```: Finite state machine implementation
- ```crl_fsm_msgs```: Message definitions for crl_fsm

### Humanoid Common Packages
- ```crl_humanoid_msgs```: Message definitions for crl_humanoid packages
- ```crl_humanoid_commons```: Common interfaces and utilities
- ```crl_humanoid_monitor```: GUI application for robot operations
- ```crl_humanoid_simulator```: Simulation tool for software-in-the-loop testing
- ```crl_humanoid_hardware```: Hardware interface for real robot communication

### G1 Controller Packages
- ```crl_g1_rlcontroller```: Reinforcement learning-based locomotion controller
- ```crl_g1_mimiccontroller```: DeepMimic and DeepTrack controllers
- ```crl_g1_goalcontroller```: Goal-directed controller with Python integration

## Build

The folder structure is the typical ROS 2 workspace layout:

```
src/
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
└── crl_g1_goalcontroller_py/
    ├── crl_g1_goalcontroller/
    └── crl_g1_goalcontroller_python/
```

Start building the workspace with sourcing the ros2 environment:
```bash
source /opt/ros/humble/setup.bash
```

In the workspace root, you can build the packages using colcon:
```bash
colcon build --cmake-args "-DCMAKE_BUILD_TYPE=Release" --symlink-install
```

You may also build specific package with its dependencies using:
```bash
colcon build --packages-select <package_name> --cmake-args "-DCMAKE_BUILD_TYPE=Release" --symlink-install
```

When building is complete, you can source the local setup files to overlay the newly built packages:

```bash
source install/setup.bash
```

## Running

### Simulator

You may start running the simulator with:

```bash
ros2 launch crl_humanoid_simulator g1.py
ros2 launch crl_g1_rlcontroller g1_sim.py
ros2 launch crl_g1_rlcontroller g1_sim_v2.py
ros2 launch crl_g1_mimiccontroller g1_sim_deepmimic.py
ros2 launch crl_g1_mimiccontroller g1_sim_deeptrack.py
ros2 launch crl_g1_goalcontroller g1_sim.py
```

### Hardware

For running on real G1 hardware:

```bash
ros2 launch crl_g1_rlcontroller g1.py
ros2 launch crl_g1_rlcontroller g1_v2.py
ros2 launch crl_g1_mimiccontroller g1_deepmimic.py
ros2 launch crl_g1_mimiccontroller g1_deeptrack.py
ros2 launch crl_g1_goalcontroller g1.py
```

