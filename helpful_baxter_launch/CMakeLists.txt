cmake_minimum_required(VERSION 2.8.3)
project(helpful_baxter_launch)

find_package(catkin REQUIRED COMPONENTS
  baxter_action_server
  baxter_moveit_config
  object_identifier
  ros_robocook
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)