cmake_minimum_required(VERSION 3.0.2)
project(tiago_move_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tiago_move_interface
 CATKIN_DEPENDS tiago_sim_hospital
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
