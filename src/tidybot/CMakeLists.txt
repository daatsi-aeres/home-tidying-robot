cmake_minimum_required(VERSION 3.14)
project(tidybot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  urdf
  worlds
  launch
  config
  maps
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/navigate.py
  scripts/ground_truth_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
