cmake_minimum_required(VERSION 2.8.3)
project(baxter_action_server)

find_package(catkin REQUIRED COMPONENTS
  moveit_core 
  moveit_ros_planning 
  moveit_ros_planning_interface 
  baxter_core_msgs
  baxter_grasps_server
  baxter_props
  baxter_tools
  message_generation
  actionlib_msgs
  std_msgs
  geometry_msgs
  baxter_control
  roscpp
  cmake_modules
  move_msgs
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED system thread)

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

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(cpp_action_server src/action_server.cpp)
add_definitions(${EIGEN_DEFINITIONS})

## Specify libraries to link a library or executable target against
target_link_libraries(cpp_action_server ${catkin_LIBRARIES} ${BOOST_LIBRARIES})
