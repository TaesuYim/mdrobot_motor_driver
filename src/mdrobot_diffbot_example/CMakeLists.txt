cmake_minimum_required(VERSION 3.16)
project(mdrobot_diffbot_example)

find_package(ament_cmake REQUIRED)

install(DIRECTORY description config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
