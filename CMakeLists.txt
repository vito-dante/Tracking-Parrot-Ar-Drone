cmake_minimum_required(VERSION 2.8)
project(seguir_objeto)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ardrone_autonomy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
