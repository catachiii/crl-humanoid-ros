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
find_package(rclcpp REQUIRED)
find_package(crl_humanoid_commons REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_ros_helper REQUIRED)
find_package(crl_humanoid_simulator REQUIRED)
find_package(crl_g1_rlcontroller REQUIRED)
# find_package(onnxruntime REQUIRED) # if you plan to link to it

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
    crl_g1_rlcontroller
)

# Find and link Eigen3 (required for CRLTron1AWalkController)
find_package(Eigen3 REQUIRED)
target_link_libraries(simulator_main Eigen3::Eigen)

# If you add a monitor_main later, enable this:
# add_executable(monitor_main src/monitor_main.cpp)
# ament_target_dependencies(monitor_main rclcpp crl_humanoid_commons crl_fsm crl_ros_helper)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME} OPTIONAL)

install(TARGETS
  simulator_main
  # monitor_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()