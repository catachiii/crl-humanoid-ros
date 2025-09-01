cmake_minimum_required(VERSION 3.11)
project(crl_humanoid_monitor)

set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crl_humanoid_commons REQUIRED)
find_package(crl_humanoid_msgs REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_ros_helper REQUIRED)

# OpenGL dependencies
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

# MuJoCo dependency - set paths manually
set(MUJOCO_INCLUDE_DIRS /opt/mujoco/include)
set(MUJOCO_LIBRARY_DIRS /opt/mujoco/lib)
set(MUJOCO_LIBRARIES mujoco)

# -----------------------------------------------------------------------------
# executable
# -----------------------------------------------------------------------------

add_executable(monitor src/main.cpp)

target_include_directories(monitor PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(monitor
    ${MUJOCO_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
)

target_link_directories(monitor PRIVATE
    ${MUJOCO_LIBRARY_DIRS}
)

# ROS2 dependencies
ament_target_dependencies(monitor
    rclcpp
    crl_humanoid_commons
    crl_humanoid_msgs
    crl_fsm
    crl_ros_helper
)

install(TARGETS monitor
    DESTINATION lib/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY config/
    DESTINATION share/${PROJECT_NAME}/config
)

# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rclcpp crl_humanoid_commons crl_humanoid_msgs crl_fsm crl_ros_helper OpenGL glfw3)
ament_package()
