cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_humanoid_commons)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# data folder path
set(
        CRL_HUMANOID_COMMONS_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
)

# -----------------------------------------------------------------------------
# thirdparty dependencies
# -----------------------------------------------------------------------------

add_subdirectory(ext)

# -----------------------------------------------------------------------------
# ros dependencies
# -----------------------------------------------------------------------------

find_package(ament_cmake 0.1 REQUIRED)
find_package(rclcpp 0.1 REQUIRED)
find_package(std_srvs 0.1 REQUIRED)
find_package(crl_humanoid_msgs 0.2 REQUIRED)
find_package(crl_fsm 0.1 REQUIRED)
find_package(crl_ros_helper 0.1 REQUIRED)
find_package(optitrack_msgs REQUIRED)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# source code files
set(CRL_TARGET_NAME ${PROJECT_NAME}_lib)

file(
        GLOB CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
        "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)

# build target
add_library(${CRL_TARGET_NAME} ${CRL_SOURCES})

# cmake target dependencies
add_dependencies(
        ${CRL_TARGET_NAME}
        "crl-utils" #
)

# target include dirs
target_include_directories(
        ${CRL_TARGET_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>" #
        "$<INSTALL_INTERFACE:include>"
)

# target link libs
target_link_libraries(
        ${CRL_TARGET_NAME}
        "crl-utils" #
)

# compile definitions
target_compile_definitions(
        ${CRL_TARGET_NAME}
        PUBLIC "CRL_HUMANOID_COMMONS_DATA_FOLDER=${CRL_HUMANOID_COMMONS_DATA_FOLDER}" #
)

# add ros dependencies
ament_target_dependencies(${CRL_TARGET_NAME} rclcpp std_srvs crl_humanoid_msgs crl_fsm crl_ros_helper optitrack_msgs)

install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
        DESTINATION include
)

install(
        TARGETS ${CRL_TARGET_NAME}
                crl-utils
        EXPORT ${CRL_TARGET_NAME}Targets
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
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
        crl_humanoid_msgs rclcpp std_srvs crl_fsm crl_ros_helper optitrack_msgs         # ros packages
        Eigen3 nlohmann_json                      # crl-loco dependencies
)

ament_package()
