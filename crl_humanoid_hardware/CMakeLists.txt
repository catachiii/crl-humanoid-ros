cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_humanoid_hardware)

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
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(crl_humanoid_commons REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_fsm_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Build unitree_sdk2 as a subdirectory
add_subdirectory(ext/unitree_sdk2)

# -----------------------------------------------------------------------------
# resource files
# -----------------------------------------------------------------------------

# launch
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# config
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# Install headers
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
)

# executable
add_executable(hardware src/main.cpp)

# Include directories for the executable
target_include_directories(hardware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${EIGEN3_INCLUDE_DIR}
)

# Link libraries
target_link_libraries(hardware
    unitree_sdk2
)

# Dependencies
ament_target_dependencies(hardware
    rclcpp
    std_msgs
    geometry_msgs
    crl_humanoid_commons
    crl_fsm
    crl_fsm_msgs
)

install(
        TARGETS hardware
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies for other packages that might depend on this one
ament_export_dependencies(rclcpp std_msgs geometry_msgs crl_humanoid_commons crl_fsm crl_fsm_msgs)
ament_package()
