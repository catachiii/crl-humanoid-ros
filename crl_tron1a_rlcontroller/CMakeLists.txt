cmake_minimum_required(VERSION 3.8)
project(crl_tron1a_rlcontroller)

# Set CMake policy to prefer GLVND for OpenGL (suppresses warning CMP0072)
if(POLICY CMP0072)
  cmake_policy(SET CMP0072 NEW)
endif()
set(OpenGL_GL_PREFERENCE GLVND)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crl_humanoid_commons REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_ros_helper REQUIRED)
find_package(crl_humanoid_simulator REQUIRED)
find_package(crl_humanoid_hardware REQUIRED)
find_package(crl_humanoid_monitor REQUIRED)
find_package(onnxruntime 1.15.0 EXACT REQUIRED)

# OpenGL dependencies for monitor
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# MuJoCo dependency - set paths manually
set(MUJOCO_INCLUDE_DIRS /opt/mujoco/include)
set(MUJOCO_LIBRARY_DIRS /opt/mujoco/lib)
set(MUJOCO_LIBRARIES mujoco)

include_directories(include)

# Executables
add_executable(simulator_main
    src/simulator_main.cpp
    src/CRLTron1AWalkController.cpp
)
ament_target_dependencies(simulator_main
    rclcpp
    crl_humanoid_commons
    crl_fsm
    crl_ros_helper
    crl_humanoid_simulator
    onnxruntime
    ament_index_cpp
)

# Find and link Eigen3 (required for CRLTron1AWalkController)
find_package(Eigen3 REQUIRED)
target_link_libraries(simulator_main Eigen3::Eigen)

# Hardware executable
add_executable(hardware_main
    src/hardware_main.cpp
    src/CRLTron1AWalkController.cpp
)
ament_target_dependencies(hardware_main
    rclcpp
    crl_humanoid_commons
    crl_fsm
    crl_ros_helper
    crl_humanoid_simulator
    crl_humanoid_hardware
    onnxruntime
    ament_index_cpp
)
target_link_libraries(hardware_main Eigen3::Eigen)

# Monitor executable
add_executable(monitor_main src/monitor_main.cpp)

target_include_directories(monitor_main PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_INCLUDE_DIRS}
)

target_link_directories(monitor_main PRIVATE
    ${MUJOCO_LIBRARY_DIRS}
)

# Link libraries for monitor
target_link_libraries(monitor_main
    ${MUJOCO_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
)

ament_target_dependencies(monitor_main
    rclcpp
    crl_humanoid_commons
    crl_humanoid_monitor
    crl_fsm
    crl_ros_helper
)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

install(TARGETS
  simulator_main
  hardware_main
  monitor_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
