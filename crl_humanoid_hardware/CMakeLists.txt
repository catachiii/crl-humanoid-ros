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
find_package(limxsdk_lowlevel REQUIRED)

# Build unitree_sdk2 as a subdirectory
add_subdirectory(ext/unitree_sdk2)

# Also find the package for proper exports
find_package(unitree_sdk2 QUIET)
if(NOT unitree_sdk2_FOUND)
    message(STATUS "unitree_sdk2 package not found, using local build")
endif()

# Create a library target for this package
add_library(${PROJECT_NAME} INTERFACE)

# Make include directories available to dependent packages
target_include_directories(${PROJECT_NAME} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ext/unitree_sdk2/thirdparty/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ext/unitree_sdk2/thirdparty/include/ddscxx>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/ddscxx>
)

# Link SDK libraries to this package's interface
target_link_libraries(${PROJECT_NAME} INTERFACE unitree_sdk2 ddsc ddscxx ${limxsdk_lowlevel_LIBRARIES})

# Add ROS dependencies to the interface library
ament_target_dependencies(${PROJECT_NAME} INTERFACE
    rclcpp
    std_msgs
    geometry_msgs
    crl_humanoid_commons
    crl_fsm
    crl_fsm_msgs
    Eigen3
    limxsdk_lowlevel
)

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

# Install DDS headers from unitree_sdk2 thirdparty
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/ext/unitree_sdk2/thirdparty/include/
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
    ${PROJECT_NAME}
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

# Install the interface library and export targets
install(TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# Export the targets for other packages to use
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include include/ddscxx)

# Export dependencies for other packages that might depend on this one
ament_export_dependencies(rclcpp std_msgs geometry_msgs crl_humanoid_commons crl_fsm crl_fsm_msgs Eigen3 unitree_sdk2 limxsdk_lowlevel)
ament_package()
