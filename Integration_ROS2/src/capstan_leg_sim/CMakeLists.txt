cmake_minimum_required(VERSION 3.8)
project(capstan_leg_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(leg_controller_node src/leg_controller_node.cpp)
ament_target_dependencies(leg_controller_node rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  leg_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  urdf
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()