# crl-humanoid-ros

Code for CRL's humanoids (unitree G1, pnd adam, limx tron1)

## Dependencies

- [ROS2](https://docs.ros.org/) Humble
    - See [this](https://docs.ros.org/en/humble/Installation.html) for install ROS2 on your system.
    - Install the additional ros middleware packages for G1
    ```bash
    sudo apt install ros-humble-rmw-cyclonedds-cpp
    sudo apt install ros-humble-rosidl-generator-dds-idl
    ```
- [colcon](https://colcon.readthedocs.io/en/released/index.html)
    - For using colcon for build ROS2 packages.
    - If you have installed ROS2 already, do
    ```bash
    sudo apt update
    sudo apt install python3-colcon-common-extensions
    ```
- [Boost](https://www.boost.org/) 1.5.4
    - Note. Dependency of [unitree_legged_sdk](https://github.com/unitreerobotics/unitree_legged_sdk).
- [cxxopts](https://github.com/jarro2783/cxxopts) 3.0.0
    - For parsing main arguments
    - Install by
    ```bash
    sudo apt update
    sudo apt install libcxxopts-dev
    ```

## Subpackages

- ```crl_ros_helper```: helper functions for ros integration
- ```crl_fsm```: finite state machine implementation
- ```crl_fsm_msgs```: message definitions for crl_fsm
- ```crl_humanoid_msgs```: message definitions for crl_humanoid packages
- ```crl_humanoid_commons```: common interfaces
- ```crl_humanoid_monitor```: GUI application for robot operations
- ```crl_humanoid_simulator```: simulation tool for software-in-the-loop test
- ```unitree_ros2```
  - ```unitree_api```
  - ```unitree_go```

