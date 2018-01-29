cmake_minimum_required(VERSION 2.8.3)

project(path_transform_planner)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        geometry_msgs
        grid_map_core
        grid_map_msgs
        grid_map_ros
        nav_msgs
        opt_utils
        internal_grid_map
        )
find_package(OpenCV REQUIRED)

catkin_package(
         INCLUDE_DIRS include
         LIBRARIES ${PROJECT_NAME}
         CATKIN_DEPENDS grid_map_core grid_map_msgs grid_map_ros roscpp internal_grid_map
         DEPENDS OpenCV
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
        src/path_planning.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${Boost_LIBRARIES}
        )
set(linked_libs
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
add_executable(${PROJECT_NAME}_node src/path_transforms_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${linked_libs}
        )