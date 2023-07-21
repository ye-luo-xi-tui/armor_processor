cmake_minimum_required(VERSION 3.10)
project(armor_processor)

## Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED
        COMPONENTS
        roscpp

        rm_msgs
        tf2_eigen
        visualization_msgs
        tf2_ros
        )


catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        CATKIN_DEPENDS
        roscpp
        rm_msgs
        tf2_eigen
        visualization_msgs
        tf2_ros
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


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME} src/processor_node.cpp src/extended_kalman_filter.cpp  src/tracker.cpp src/main.cpp src/armor_filter.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
