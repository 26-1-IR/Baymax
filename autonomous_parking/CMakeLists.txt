cmake_minimum_required(VERSION 3.8)
project(autonomous_parking)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch worlds maps config models scripts urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
