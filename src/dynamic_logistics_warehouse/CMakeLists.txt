cmake_minimum_required(VERSION 3.8)
project(dynamic_logistics_warehouse)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch models worlds maps rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()