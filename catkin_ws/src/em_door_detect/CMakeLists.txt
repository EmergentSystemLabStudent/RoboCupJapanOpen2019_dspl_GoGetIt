cmake_minimum_required(VERSION 2.8.3)
project(em_door_detect)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  em_door_detect.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  sensor_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
