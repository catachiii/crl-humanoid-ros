cmake_minimum_required(VERSION 3.8)
project(natnet_sdk)

add_library(natnet_lib SHARED IMPORTED GLOBAL)
target_include_directories(natnet_lib INTERFACE include)
set_target_properties(natnet_lib PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/libNatNet.so")