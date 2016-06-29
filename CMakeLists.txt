cmake_minimum_required(VERSION 2.8.3)
project(great_ardrones)

find_package(catkin REQUIRED COMPONENTS
  ardrone_autonomy
  roscpp
  rospy
  std_msgs
  vrpn_client_ros
  tf
  geometry_msgs
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pid_controller src/pid_controller.cpp)
target_link_libraries(pid_controller ${catkin_LIBRARIES})
add_dependencies(pid_controller great_ardrones)
