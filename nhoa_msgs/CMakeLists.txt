cmake_minimum_required(VERSION 2.8.3)
project(nhoa_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED
  genmsg
  # actionlib_msgs
  # control_msgs
  # geometry_msgs
  # moveit_msgs
  std_msgs
  # trajectory_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  SmileScore.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  # control_msgs
  # geometry_msgs
  # moveit_msgs
  std_msgs
  # trajectory_msgs
)

## Generate catkin specific configuration
catkin_package(
 CATKIN_DEPENDS std_msgs # actionlib_msgs control_msgs geometry_msgs moveit_msgs  trajectory_msgs
)


