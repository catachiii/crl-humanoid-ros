# we need cmake >= 3.11 to support fetch_content that downloads dependencies
cmake_minimum_required(VERSION 3.11)

# set name of the project
project(crl_humanoid_msgs)

# set cxx standard and compiler flag
set(CMAKE_CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------------------------------------------------------
# ros
# -----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# -----------------------------------------------------------------------------
# code
# -----------------------------------------------------------------------------

# messages
set(msg_files
        "msg/Sensor.msg"
        "msg/State.msg"
        "msg/Control.msg"
        "msg/Monitor.msg"
        "msg/Remote.msg"
        "msg/ProfilingInfo.msg"
        "msg/PyState.msg"
        "msg/PyControl.msg"
)

# services
set(srv_files
        "srv/Model.srv"
        "srv/Ping.srv"
        "srv/Restart.srv"
        "srv/ElasticBand.srv"
)

# interface
rosidl_generate_interfaces(${PROJECT_NAME}
        ${msg_files}
        ${srv_files}
        DEPENDENCIES
        sensor_msgs
        geometry_msgs
        trajectory_msgs
)

# this should be the last call
ament_export_dependencies(rosidl_default_runtime)
ament_package()
