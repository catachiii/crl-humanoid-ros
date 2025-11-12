cmake_minimum_required(VERSION 3.8)
project(optitrack_adaptor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(optitrack_msgs REQUIRED)

add_subdirectory(ext)

add_library(adaptor src/NatNetPublisher.cpp src/Dispatcher.cpp)
target_link_libraries(adaptor PUBLIC natnet_lib)
ament_target_dependencies(adaptor PUBLIC rclcpp optitrack_msgs)
target_include_directories(adaptor PUBLIC 
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)

add_executable(adaptor_runner src/adaptor_runner.cpp)
target_link_libraries(adaptor_runner adaptor)

install(TARGETS
  adaptor_runner
  RUNTIME DESTINATION lib/${PROJECT_NAME})

get_target_property(DLL_LOC natnet_lib IMPORTED_LOCATION)
install(FILES ${DLL_LOC} DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
