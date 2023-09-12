cmake_minimum_required(VERSION 3.0.2)
project(local_map_creater)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs
nav_msgs
sensor_msgs
geometry_msgs
tf
tf2
# roomba_500driver_meiji
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES local_map_creater
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME} src/local_map_creater.cpp)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/local_map_creater_node.cpp)

## Add cmake target dependencies of the executable
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
