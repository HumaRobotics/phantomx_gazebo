cmake_minimum_required(VERSION 2.8.3)
project(phantomx_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS 
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  gazebo_ros  
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS 
    sensor_msgs
    gazebo_ros 
    hector_gazebo_plugins
  DEPENDS 
    gazebo
)