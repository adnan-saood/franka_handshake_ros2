cmake_minimum_required(VERSION 3.8)
project(franka_handshake_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Handshake.action"
    DEPENDENCIES builtin_interfaces
)

ament_package()