cmake_minimum_required(VERSION 2.8.3)
project(manipulator_interpolation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  nav_msgs
  std_msgs
  orocos_kdl
  kdl_parser
)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
   frame_pose.msg
   frame_vel.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   jint_control_srv.srv
   oint_control_srv.srv
   joint_state.srv
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   sensor_msgs
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
#  LIBRARIES manipulator_interpolation
  CATKIN_DEPENDS
  geometry_msgs
  nav_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(jint src/jint.cpp)
target_link_libraries(jint ${catkin_LIBRARIES})
add_dependencies(jint manipulator_interpolation_generate_messages_cpp)

add_executable(oint src/oint.cpp)
target_link_libraries(oint ${catkin_LIBRARIES})
add_dependencies(oint manipulator_interpolation_generate_messages_cpp)

add_executable(kinematic src/kinematic.cpp)
target_link_libraries(kinematic ${catkin_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${orocos_kdl_LIBRARIES})
add_dependencies(kinematic manipulator_interpolation_generate_messages_cpp)