cmake_minimum_required(VERSION 2.8.3)

project(coverage_exploration)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra ${CMAKE_CXX_FLAGS}")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  tf
  visualization_msgs
  geometry_msgs
)

find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(EIGEN3 REQUIRED eigen3)
    set(EIGEN3_INCLUDE_DIR ${EIGEN3_INCLUDE_DIRS})
endif()
add_definitions(${EIGEN_DEFINITIONS})

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roscpp
#  CATKIN_DEPENDS test
   DEPENDS  Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
   src/Planner.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )

set(linked_libs
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_node src/exploration_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
        ${linked_libs}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/roscpp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
