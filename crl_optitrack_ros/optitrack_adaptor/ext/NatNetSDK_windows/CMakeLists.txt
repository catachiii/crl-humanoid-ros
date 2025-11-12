cmake_minimum_required(VERSION 3.8)
project(natnet_sdk)

add_library(natnet_lib SHARED IMPORTED GLOBAL)
target_include_directories(natnet_lib INTERFACE include)

if(CMAKE_SIZEOF_VOID_P EQUAL 4)
  set_target_properties(natnet_lib PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/x86/NatNetLib.dll"
                                              IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/x86/NatNetLib.lib")
else()
set_target_properties(natnet_lib PROPERTIES IMPORTED_LOCATION "${CMAKE_CURRENT_LIST_DIR}/lib/x64/NatNetLib.dll"
                                            IMPORTED_IMPLIB "${CMAKE_CURRENT_LIST_DIR}/lib/x64/NatNetLib.lib")
endif()
