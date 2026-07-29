cmake_minimum_required(VERSION 3.8)
project(air_hockey_robot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PuckState.msg"
  "msg/PuckDetection.msg" 
  "msg/PredictedEntry.msg" 
  DEPENDENCIES geometry_msgs sensor_msgs
)

ament_package()