cmake_minimum_required(VERSION 2.8.3)
project(em_tf2)

find_package(catkin REQUIRED COMPONENTS
rospy
geometry_msgs
message_generation
)

add_service_files(
  FILES
  em_tf2.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
