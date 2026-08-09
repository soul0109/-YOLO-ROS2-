cmake_minimum_required(VERSION 3.8)
project(navigation_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config maps launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
