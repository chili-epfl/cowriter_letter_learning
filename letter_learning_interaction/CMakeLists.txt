cmake_minimum_required(VERSION 2.8.3)
project(letter_learning_interaction)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(
   FILES
   clearAllShapes.srv
   displayNewShape.srv
   isPossibleToDisplayNewShape.srv
   indexOfLocation.srv
   shapeAtLocation.srv
   closestShapesToLocation.srv
   displayShapeAtLocation.srv
)

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   Shape.msg
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
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 CATKIN_DEPENDS message_runtime 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
 install(PROGRAMS
   nodes/display_manager_server.py
   nodes/word_card_detector.py
   nodes/tablet_input_interpreter.py
   nodes/learning_words_nao.py
   nodes/watchdog_clearer.py
   nodes/activity_manager.py
   nodes/drawing_nao.py
   nodes/log.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY include/
   DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)
## Mark other files for installation (e.g. launch and bag files, etc.) 
 install(FILES
launch/nao_learning.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 )

