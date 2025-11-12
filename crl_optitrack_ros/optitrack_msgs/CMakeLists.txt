cmake_minimum_required(VERSION 3.8)
project(optitrack_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# topics
set(msg_files 
        "msg/AnalogChannelData.msg"
        "msg/DeviceData.msg"
        "msg/ForcePlateData.msg"
        "msg/LabelledMarkerData.msg"
        "msg/MarkerData.msg"
        "msg/MarkerSetData.msg"
        "msg/MocapFrameData.msg"
        "msg/RigidbodyData.msg"
        "msg/SkeletonData.msg"
)

# services
set(srv_files
        "srv/Connect.srv"
        "srv/Disconnect.srv"
        "srv/Reset.srv"
)

# interface
rosidl_generate_interfaces(${PROJECT_NAME} 
        ${msg_files}
        ${srv_files}
        DEPENDENCIES 
        builtin_interfaces
)

# this should be the last call
ament_export_dependencies(rosidl_default_runtime builtin_interfaces)
ament_package()

