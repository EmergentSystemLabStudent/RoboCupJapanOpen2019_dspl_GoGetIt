cmake_minimum_required(VERSION 2.8.3)
project(em_follow_me)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
  actionlib_msgs
)

add_service_files(
  FILES
  follow_me.srv
)

add_action_files(
  DIRECTORY
  action
  FILES
  follow_me.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_runtime
  actionlib_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
