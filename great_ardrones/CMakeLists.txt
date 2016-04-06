cmake_minimum_required(VERSION 2.8.3)
project(great_ardrones)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
  rospy
  std_msgs
  vrpn_client_ros
)
