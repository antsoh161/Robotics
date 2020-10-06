cmake_minimum_required(VERSION 2.8.3)
project(lab2_publisher)
add_compile_options(-g -std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
  urdf
  urdf_parser_plugin
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lab2_publisher
  CATKIN_DEPENDS eigen_conversions geometry_msgs roscpp rospy std_msgs tf urdf urdf_parser_plugin
#  DEPENDS system_lib
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
add_library(antonsRoboLib
   		src/homtra.cpp
   		src/joint.cpp
   		src/link.cpp
		src/chain.cpp
		src/FKParser.cpp
)

add_executable(${PROJECT_NAME}_node src/lab2_publisher_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
    antonsRoboLib
    ${catkin_LIBRARIES}
)

