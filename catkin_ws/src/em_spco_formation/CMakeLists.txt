cmake_minimum_required(VERSION 2.8.3)
project(em_spco_formation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
  message_generation
)

add_service_files(
  FILES
  spcof_learn.srv
  spcof_data.srv
  spcof_image.srv
  spcof_yolo.srv
  spcof_pose.srv
  spcof_word.srv
  spcof_place2name.srv
  spcof_name2place.srv
  spcof_rviz.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  geometry_msgs
  visualization_msgs
  message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
