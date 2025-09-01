
cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_g1_rlcontroller_commons)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# data folder path
set(
        CRL_G1_RLCONTROLLER_COMMONS_DATA_FOLDER
        "\"${CMAKE_CURRENT_LIST_DIR}/data\""
        CACHE STRING ""
)

# -----------------------------------------------------------------------------
# ros dependencies
# -----------------------------------------------------------------------------

find_package(crl_humanoid_commons 0.8.0 EXACT REQUIRED)
find_package(onnxruntime 1.15.0 EXACT REQUIRED)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# source code files
set(CRL_TARGET_NAME ${PROJECT_NAME}_lib)

file(
        GLOB CRL_SOURCES #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# build target
add_library(${CRL_TARGET_NAME} ${CRL_SOURCES})

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
        PUBLIC "CRL_G1_RLCONTROLLER_COMMONS_DATA_FOLDER=${CRL_G1_RLCONTROLLER_COMMONS_DATA_FOLDER}" #
)

# add ros dependencies
ament_target_dependencies(${CRL_TARGET_NAME} crl_humanoid_commons onnxruntime)

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
        crl_humanoid_commons onnxruntime
)

ament_package()
