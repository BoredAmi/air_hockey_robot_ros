cmake_minimum_required(VERSION 3.8)
project(pc_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(air_hockey_robot_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${air_hockey_robot_msgs_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/egm
)

# ==========================================
# 1. PERCEPTION NODE
# ==========================================
add_executable(perception_node app/main.cpp src/perception_node.cpp)
target_link_libraries(perception_node
  rclcpp::rclcpp
  rclcpp_components::component
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(perception_node rclcpp air_hockey_robot_msgs)

install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# 2. CONFIG TUNER
# ==========================================
add_executable(config_tuner
  app/config_tuner_main.cpp
  src/camera_bridge.cpp
)
target_link_libraries(config_tuner
  rclcpp::rclcpp
  cv_bridge::cv_bridge
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(config_tuner rclcpp air_hockey_robot_msgs)

install(TARGETS config_tuner 
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# 3. TRAJECTORY NODE
# ==========================================
add_executable(trajectory_node
  app/trajectory_node_main.cpp
  src/trajectory_node.cpp
  src/trajectory.cpp
  src/kalman.cpp
)
target_link_libraries(trajectory_node 
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  air_hockey_robot_msgs::air_hockey_robot_msgs__rosidl_typesupport_cpp   # ADD THIS
)
ament_target_dependencies(trajectory_node rclcpp air_hockey_robot_msgs)

install(TARGETS trajectory_node 
  DESTINATION lib/${PROJECT_NAME}
)

# ==========================================
# 4. MOVEMENT NODE
# ==========================================

find_package(Protobuf REQUIRED)

set(EGM_PROTO ${CMAKE_CURRENT_SOURCE_DIR}/egm/egm.proto)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/egm/egm.pb.cc)
    add_library(egm_proto STATIC ${CMAKE_CURRENT_SOURCE_DIR}/egm/egm.pb.cc)
    target_include_directories(egm_proto PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/egm ${Protobuf_INCLUDE_DIRS})
    target_link_libraries(egm_proto PUBLIC ${Protobuf_LIBRARIES} ${OpenCV_LIBRARIES})
else()
    protobuf_generate_cpp(EGM_SRCS EGM_HDRS ${EGM_PROTO})
    add_library(egm_proto STATIC ${EGM_SRCS} ${EGM_HDRS})
    target_include_directories(egm_proto PUBLIC ${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIRS})
    target_link_libraries(egm_proto PUBLIC ${Protobuf_LIBRARIES} ${OpenCV_LIBRARIES})
endif()

add_executable(movement_node
  app/movement_node_main.cpp
  src/movement_node.cpp
  src/movement.cpp
)

ament_target_dependencies(movement_node rclcpp air_hockey_robot_msgs)

target_link_libraries(movement_node egm_proto)
install(TARGETS movement_node
  DESTINATION lib/${PROJECT_NAME}
)
# ==========================================
# 5. Very nice monitor for kalman filter
# ==========================================
add_executable(kalman_monitor app/kalman_monitor_main.cpp)
ament_target_dependencies(kalman_monitor rclcpp air_hockey_robot_msgs)
target_link_libraries(kalman_monitor ${OpenCV_LIBRARIES})
install(TARGETS kalman_monitor DESTINATION lib/${PROJECT_NAME})

ament_package()