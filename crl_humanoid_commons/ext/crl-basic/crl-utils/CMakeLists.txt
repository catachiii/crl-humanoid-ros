cmake_minimum_required(VERSION 3.11)

project(crl-utils)

set(CMAKE_CXX_STANDARD 17)
set(CRL_TARGET_NAME ${PROJECT_NAME})

# dependencies
find_package(Eigen3 3.4 REQUIRED)
find_package(nlohmann_json 3.10 REQUIRED)

# files
file(
        GLOB CRL_SOURCES #
        "include/crl-basic/utils/*.h" #
        "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp" #
)

# target include dirs
list(
        APPEND CRL_TARGET_INCLUDE_DIRS #
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/>" #
        "$<INSTALL_INTERFACE:include>" #
)

# target link libs
list(
        APPEND CRL_TARGET_LINK_LIBS #
        "Eigen3::Eigen" #
        "nlohmann_json::nlohmann_json" #
)

# compile definitions
list(
        APPEND CRL_COMPILE_DEFINITIONS #
        "CRL_LOCO_DATA_FOLDER=${CRL_LOCO_DATA_FOLDER}" #
)

# create target
create_crl_library(
        ${CRL_TARGET_NAME}
        "${CRL_SOURCES}" #
        "${CRL_TARGET_DEPENDENCIES}" #
        "${CRL_TARGET_INCLUDE_DIRS}" #
        "${CRL_TARGET_LINK_LIBS}" #
        "${CRL_COMPILE_DEFINITIONS}" #
)
