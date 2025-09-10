cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_g1_rlcontroller)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# data folder path
set(
        CRL_G1_RLCONTROLLER_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
)

# -----------------------------------------------------------------------------
# ros dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(crl_humanoid_commons 0.8.0 EXACT REQUIRED)
find_package(onnxruntime 1.15.0 EXACT REQUIRED)
find_package(crl_humanoid_simulator 0.8.0 EXACT REQUIRED)
find_package(crl_humanoid_monitor 0.2.0 EXACT REQUIRED)
find_package(crl_humanoid_hardware REQUIRED)
find_package(crl_fsm REQUIRED)
find_package(crl_ros_helper REQUIRED)

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

# data
install(DIRECTORY data DESTINATION share/${PROJECT_NAME}/)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# library source code files
set(CRL_TARGET_NAME ${PROJECT_NAME}_lib)

file(
        GLOB CRL_LIBRARY_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/CRLG1WalkController.cpp" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/CRLG1GetUpController.cpp" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/CRLG1SitDownController.cpp" #
)

# build library target
add_library(${CRL_TARGET_NAME} ${CRL_LIBRARY_SOURCES})

# target include dirs
target_include_directories(
        ${CRL_TARGET_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" #
        "$<INSTALL_INTERFACE:include>"
)

# compile definitions
target_compile_definitions(
        ${CRL_TARGET_NAME}
        PUBLIC "CRL_G1_RLCONTROLLER_DATA_FOLDER=${CRL_G1_RLCONTROLLER_DATA_FOLDER}" #
)

# add ros dependencies to library
ament_target_dependencies(${CRL_TARGET_NAME} crl_humanoid_commons onnxruntime)

# -----------------------------------------------------------------------------
# executables
# -----------------------------------------------------------------------------

# simulator executable
add_executable(simulator_main src/simulator_main.cpp)

ament_target_dependencies(simulator_main crl_humanoid_simulator crl_humanoid_commons onnxruntime)
target_link_libraries(simulator_main ${CRL_TARGET_NAME})

# hardware executable
add_executable(hardware_main src/hardware_main.cpp)

ament_target_dependencies(hardware_main crl_humanoid_hardware crl_humanoid_commons onnxruntime)
target_link_libraries(hardware_main ${CRL_TARGET_NAME})

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

# simulator_main_v2 executable
add_executable(simulator_main_v2 src/simulator_main_v2.cpp)

ament_target_dependencies(simulator_main_v2 crl_humanoid_simulator crl_humanoid_commons onnxruntime crl_fsm)
target_link_libraries(simulator_main_v2 ${CRL_TARGET_NAME})

# monitor_main_v2 executable
add_executable(monitor_main_v2 src/monitor_main_v2.cpp)

target_include_directories(monitor_main_v2 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${MUJOCO_INCLUDE_DIRS}
)

target_link_directories(monitor_main_v2 PRIVATE
    ${MUJOCO_LIBRARY_DIRS}
)

# Link libraries for monitor_main_v2
target_link_libraries(monitor_main_v2
    ${MUJOCO_LIBRARIES}
    ${OPENGL_LIBRARIES}
    glfw
)

# ROS2 dependencies for monitor_main_v2
ament_target_dependencies(monitor_main_v2
    rclcpp
    crl_humanoid_commons
    crl_humanoid_monitor
    crl_fsm
    crl_ros_helper
)

# hardware_main_v2 executable
add_executable(hardware_main_v2 src/hardware_main_v2.cpp)

ament_target_dependencies(hardware_main_v2 crl_humanoid_hardware crl_humanoid_commons onnxruntime crl_fsm)
target_link_libraries(hardware_main_v2 ${CRL_TARGET_NAME})

# installations
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
)

install(
        TARGETS ${CRL_TARGET_NAME}
        EXPORT ${CRL_TARGET_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(
        TARGETS simulator_main hardware_main monitor_main simulator_main_v2 monitor_main_v2 hardware_main_v2
        DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(${CRL_TARGET_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        crl_humanoid_commons onnxruntime crl_humanoid_simulator crl_humanoid_hardware crl_humanoid_monitor crl_fsm crl_ros_helper OpenGL glfw3
)

ament_package()
