cmake_minimum_required(VERSION 2.8.3)
project(ros_mppt)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  std_msgs
)

########################################
## ROS messages, services and actions ##
########################################

## Message files
add_message_files(
   FILES
   mppt.msg
)

## Generate added messages dependecies
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS message_runtime
  CATKIN_DEPENDS rospy
  CATKIN_DEPENDS std_msgs
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
