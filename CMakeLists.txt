cmake_minimum_required(VERSION 2.8.3)
project(pole_based_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	rospy
	sensor_msgs
	std_msgs
	visualization_msgs
	roscpp
    geodesy
	pcl_ros
	pcl_conversions
	message_generation
	tf2
	tf2_ros
	tf2_msgs
    dynamic_reconfigure
)

find_package(PCL REQUIRED COMPONENTS common search sample_consensus segmentation registration recognition)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"

##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
    FILES
    Pole.msg
    PoleArray.msg
    PoleMatch.msg
    PoleMatchArray.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs#   sensor_msgs#   std_msgs#   visualization_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )
generate_dynamic_reconfigure_options(
  cfg/pole_detection.cfg
  cfg/pole_recognition.cfg
  cfg/pole_position_correction.cfg
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
	CATKIN_DEPENDS
	message_runtime
	sensor_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
	${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/pole_based_localization.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Declare a C++ executable
add_executable(${PROJECT_NAME}
	src/main.cpp
)

add_executable(${PROJECT_NAME}_recognizer
	src/pole_recognition/main.cpp
)

add_executable(map_to_utm_transform_broadcaster
	src/utm_to_map/utm_map_transform_broadcaster.cpp
)


add_executable(${PROJECT_NAME}_poscorrection
	src/position_correction_publisher/main.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
	${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_recognizer
	${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_poscorrection
	${catkin_LIBRARIES}
)

target_link_libraries(map_to_utm_transform_broadcaster
	${catkin_LIBRARIES}
)


## Declare a C++ library
add_library(${PROJECT_NAME}_nodelet
	src/SimplePoleDetector.cpp
)

add_dependencies(${PROJECT_NAME}_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)

add_library(${PROJECT_NAME}_recognizer_nodelet
	src/pole_recognition/Recognizer.cpp
)

add_dependencies(${PROJECT_NAME}_recognizer_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)

# make sure configure headers are built before any node using them
add_dependencies(${PROJECT_NAME}_recognizer_nodelet ${PROJECT_NAME}_gencfg)

add_library(${PROJECT_NAME}_poscorrect_nodelet
	src/position_correction_publisher/position_correction_publisher.cpp
)

add_dependencies(${PROJECT_NAME}_poscorrect_nodelet
	${PROJECT_NAME}_generate_messages_cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
	${catkin_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_recognizer_nodelet
	${catkin_LIBRARIES}
	sqlite3
	pcl_registration
)

target_link_libraries(${PROJECT_NAME}_poscorrect_nodelet
	${catkin_LIBRARIES}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(
	TARGETS
		${PROJECT_NAME}
		${PROJECT_NAME}_nodelet
		${PROJECT_NAME}_recognizer
		${PROJECT_NAME}_recognizer_nodelet
		${PROJECT_NAME}_poscorrection
		${PROJECT_NAME}_poscorrect_nodelet
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(
	DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
	FILES nodelet_plugins.xml
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pole_based_localization.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
