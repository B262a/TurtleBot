cmake_minimum_required(VERSION 2.8.3)
project(vision)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
message("---- OpenCV ----")
message(${OpenCV_CONFIG})




###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(vision_node src/thermalfollowerforturtlesim.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vision_node
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}

)
