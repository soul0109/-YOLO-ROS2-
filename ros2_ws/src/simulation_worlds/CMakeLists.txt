cmake_minimum_required(VERSION 3.8)
project(simulation_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds models launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(PROGRAMS
  scripts/cmd_vel_timeout.py
  scripts/keyboard_teleop.py
  scripts/test_cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
