cmake_minimum_required(VERSION 3.0.2)
project(armpap_control)


find_package(catkin REQUIRED COMPONENTS
  controller_manager
  effort_controllers
  joint_state_controller
  robot_state_publisher
  rospy
)
catkin_package()

include_directories( ${catkin_INCLUDE_DIRS})


