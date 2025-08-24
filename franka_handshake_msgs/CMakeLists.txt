cmake_minimum_required(VERSION 3.8)
# Suppress CMP0002 warning
option(CHECK_TIDY "Enable clang-tidy checks" OFF)

project(franka_handshake_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "action/Handshake.action"
    "action/SetGains.action"
    DEPENDENCIES std_msgs geometry_msgs sensor_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
