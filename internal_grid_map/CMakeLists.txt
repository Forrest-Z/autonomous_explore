cmake_minimum_required(VERSION 2.8.3)

project(internal_grid_map)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(catkin_deps
        roscpp
        roslib
        cv_bridge
        geometry_msgs
        grid_map_core
        grid_map_ros
        grid_map_msgs
        grid_map_cv
        opt_utils
        nav_msgs)

find_package(catkin REQUIRED COMPONENTS
    ${catkin_deps})

find_package(OpenCV 3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${catkin_deps}
    DEPENDS OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_BIN_DESTINATION})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/data
  DESTINATION ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_SHARE_DESTINATION})

## Declare a C++ library
add_library(${PROJECT_NAME}
        src/internal_grid_map.cpp)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})

add_executable(grid_map_demo
        src/main.cpp)

target_link_libraries(grid_map_demo
        ${catkin_LIBRARIES}
        ${PROJECT_NAME})

add_executable(submap_node
  src/submap_node.cpp)
target_link_libraries(submap_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME})

catkin_add_gtest(internal_grid_map_test
        tests/internal_grid_map_test.cpp)

target_link_libraries(internal_grid_map_test
        ${catkin_LIBRARIES}
        ${PROJECT_NAME})
