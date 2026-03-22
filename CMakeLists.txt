cmake_minimum_required(VERSION 3.8)
project(embedded_systems_project)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
