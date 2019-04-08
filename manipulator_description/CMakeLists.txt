cmake_minimum_required(VERSION 2.8.3)
project(manipulator_description)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  tf
  urdf
  orocos_kdl
  kdl_parser
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   DhToRPY.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES manipulator_description
  CATKIN_DEPENDS 
  roscpp 
  rospy
  std_msgs
  geometry_msgs
  message_runtime
  tf
  urdf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)

add_executable(dh_to_rpy_server src/dh_to_rpy_server.cpp)
target_link_libraries(dh_to_rpy_server ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(dh_to_rpy_server manipulator_description_generate_messages_cpp)

add_executable(dh_to_rpy_client src/dh_to_rpy_client.cpp)
target_link_libraries(dh_to_rpy_client ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(dh_to_rpy_client manipulator_description_generate_messages_cpp)