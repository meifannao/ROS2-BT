cmake_minimum_required(VERSION 3.16)
project(btcpp_ros2_interfaces)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(btcpp_ros2_interfaces
    "msg/NodeStatus.msg"
    "action/ExecuteTree.action"
    "action/Sleep.action"
    "action/Photo.action")

ament_export_dependencies(rosidl_default_runtime)
ament_package()
