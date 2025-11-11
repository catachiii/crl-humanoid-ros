cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_g1_goalcontroller)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# ros dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crl_humanoid_commons REQUIRED)
find_package(crl_humanoid_simulator REQUIRED)
find_package(crl_humanoid_hardware REQUIRED)
find_package(crl_humanoid_monitor REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_ros_helper REQUIRED)
find_package(crl_g1_rlcontroller REQUIRED)

# -----------------------------------------------------------------------------
# external dependencies for monitor
# -----------------------------------------------------------------------------

# OpenGL dependencies for monitor
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# MuJoCo dependency - set paths manually
set(MUJOCO_INCLUDE_DIRS /opt/mujoco/include)
set(MUJOCO_LIBRARY_DIRS /opt/mujoco/lib)
set(MUJOCO_LIBRARIES mujoco)

# -----------------------------------------------------------------------------
# resource files
# -----------------------------------------------------------------------------

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# -----------------------------------------------------------------------------
# executables
# -----------------------------------------------------------------------------

# simulator executable
add_executable(simulator_main src/simulator_main.cpp)

ament_target_dependencies(simulator_main
    crl_humanoid_simulator
    crl_humanoid_commons
    crl_g1_rlcontroller
    crl_fsm
)

# monitor executable
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

# ROS2 dependencies for monitor
ament_target_dependencies(monitor_main
    rclcpp
    crl_humanoid_commons
    crl_humanoid_monitor
    crl_fsm
    crl_ros_helper
)

# hardware executable
add_executable(hardware_main src/hardware_main.cpp)

ament_target_dependencies(hardware_main
    crl_humanoid_hardware
    crl_humanoid_commons
    crl_g1_rlcontroller
    crl_fsm
)

# installations
install(
    TARGETS simulator_main monitor_main hardware_main
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(
    crl_humanoid_commons
    crl_humanoid_simulator
    crl_humanoid_hardware
    crl_humanoid_monitor
    crl_fsm
    crl_ros_helper
    crl_g1_rlcontroller
    OpenGL
    glfw3
)

ament_package()
