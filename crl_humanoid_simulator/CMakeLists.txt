cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_humanoid_simulator)

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
find_package(crl_humanoid_commons 0.8 REQUIRED EXACT)
find_package(crl_humanoid_msgs REQUIRED)
find_package(crl_fsm REQUIRED)

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
# code
# -----------------------------------------------------------------------------

# SimNode is a template class defined entirely in the header
# No library target needed

# Install headers
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
)

# executable
add_executable(sim src/main.cpp)

# Include directories for the executable
target_include_directories(sim PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(sim
    ${MUJOCO_LIBRARIES}
)
target_link_directories(sim PRIVATE ${MUJOCO_LIBRARY_DIRS})

# Dependencies
ament_target_dependencies(sim
    rclcpp
    crl_humanoid_commons
    crl_humanoid_msgs
    crl_fsm
)

install(
        TARGETS sim
        DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# -----------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export own include directory + MuJoCo includes
ament_export_include_directories(
  include
  ${MUJOCO_INCLUDE_DIRS}
)

ament_export_libraries(${MUJOCO_LIBRARIES})

ament_export_dependencies(
  rclcpp
  crl_humanoid_commons
  crl_humanoid_msgs
  crl_fsm
)

ament_package()
