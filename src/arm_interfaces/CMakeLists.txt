cmake_minimum_required(VERSION 3.8)
project(arm_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/JointCommand.msg"
  "srv/PickPlace.srv"
  "srv/DetectObjects.srv"
  "srv/GetJointStates.srv"
  "action/ExecuteTask.action"
  DEPENDENCIES geometry_msgs std_msgs sensor_msgs
)

ament_package()
