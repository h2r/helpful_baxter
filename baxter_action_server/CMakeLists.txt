cmake_minimum_required(VERSION 2.8.3)
project(baxter_action_server)

find_package(catkin REQUIRED COMPONENTS
  baxter_core_msgs
  baxter_grasps_server
  baxter_props
  baxter_tools
  message_generation
  actionlib_msgs
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   BaxterAction.msg
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   move_msgs
   actionlib_msgs
 )

catkin_package()