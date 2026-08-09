cmake_minimum_required(VERSION 3.8)
project(simulation_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
