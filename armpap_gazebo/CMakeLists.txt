cmake_minimum_required(VERSION 3.0.2)
project(armpap_gazebo)


find_package(catkin REQUIRED COMPONENTS
  armpap_description
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  xacro
)


catkin_package()


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


