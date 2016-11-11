cmake_minimum_required(VERSION 2.8)
project(Tracking_Parrot_Ar_Drone)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  ardrone_autonomy
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
