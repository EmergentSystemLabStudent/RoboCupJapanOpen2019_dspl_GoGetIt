cmake_minimum_required(VERSION 2.8.3)
project(em_map_record)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  message_generation
)

add_service_files(
  FILES
  map_image.srv
  map_pose.srv
  map_record.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  sensor_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
