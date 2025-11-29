cmake_minimum_required(VERSION 3.5)
project(obstacle_avoider)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(obstacle_avoider_node src/obstacle_avoider.cpp)
ament_target_dependencies(obstacle_avoider_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  obstacle_avoider_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
