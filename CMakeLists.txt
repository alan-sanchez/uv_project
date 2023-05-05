cmake_minimum_required(VERSION 3.0.2)
project(uv_project)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  octomap_ros
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(PCL REQUIRED COMPONENTS common io)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_msgs REQUIRED)



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
  INCLUDE_DIRS include
  LIBRARIES uv_project
  CATKIN_DEPENDS roscpp rospy std_msgs pcls_msgs octomap_msgs octomap_ros
  DEPENDS EIGEN3 PCL octomap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${octomap_msgs_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)



#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

## Mark cpp header files for installation
#install(DIRECTORY include/${PROJECT_NAME}/
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
#)


#############
## Testing ##
#############

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

add_executable(accumulation
  src/accumulation.cpp
  src/uv_model.cpp
  src/in_polygon_check.cpp
)
target_link_libraries(accumulation
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)
