cmake_minimum_required(VERSION 3.5)
project(nhoa_bt)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_compile_options(-g)

## is used, also find other catkin packages
if(POLICY CMP0057)
    cmake_policy(SET CMP0057 NEW)
endif()

find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  control_msgs 
  controller_manager_msgs
  play_motion_msgs 
  moveit_ros_planning_interface 
  roscpp 
  sensor_msgs
  diagnostic_msgs 
  diagnostic_updater 
  behaviortree_cpp_v3)
find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(INCLUDE_DIRS include
               LIBRARIES play_motion_helpers
               CATKIN_DEPENDS actionlib control_msgs controller_manager_msgs
               play_motion_msgs moveit_ros_planning_interface roscpp sensor_msgs)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include/nhoa_bt
  include/bt_action_nodes_h
  include/bt_condition_nodes_h)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})

link_directories(
  include/nhoa_bt
  include/bt_action_nodes_h
  include/bt_condition_nodes_h
)

# BT NODES
add_executable(${PROJECT_NAME}_demo src/${PROJECT_NAME}_demo.cpp 
               # BT ACTION NODES.
               src/bt_action_nodes/ExecuteMotion.cpp
               # BT CONDITION NODES.
               #############################
               # 
               src/plan_motion.cpp
               )
add_dependencies(${PROJECT_NAME}_demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_demo ${catkin_LIBRARIES})
