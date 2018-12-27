cmake_minimum_required(VERSION 2.8.3)
project(handson_gazebo)

find_package(catkin REQUIRED)
catkin_package(
  CATKIN_DEPENDS xacro ur_description handson_description
)