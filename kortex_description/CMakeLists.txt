cmake_minimum_required(VERSION 3.10.2)
project(kortex_description)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY gen3_arm gripper_robotiq_2f_85 robots rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
