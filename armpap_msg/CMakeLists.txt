cmake_minimum_required(VERSION 3.0.2)
project(armpap_msg)


find_package(
  catkin REQUIRED COMPONENTS
  sensor_msgs
  genmsg
  message_generation 
  std_msgs
  
)
## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   cmd_robot_msg.msg
   armpap_msg.msg
  )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   
  )

catkin_package(
  # INCLUDE_DIRS include
  CATKIN_DEPENDS
  message_runtime 
)




