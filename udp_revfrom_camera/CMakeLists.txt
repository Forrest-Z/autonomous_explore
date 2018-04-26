cmake_minimum_required(VERSION 2.8.3)

project(udp_revfrom_camera)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-Wall -Wextra -fPIC ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  iv_explore_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -I /usr/local/include/data_transmission")

find_library(EXT_LIBRARY
        NAMES data_transmission
        PATHS "/usr/local/lib/data_transmission/"
        NO_DEFAULT_PATH)

find_library(EXT_BOOST_LIBRARY
        NAMES boostudp
        PATHS "/usr/local/lib/data_transmission/"
        NO_DEFAULT_PATH)
if(EXT_LIBRARY)
    # Multiple CMake projects cases (i.e. 'catkin build'):
    # - The target has already been built when its dependencies require it
    # - Specify full path to found library
    list(APPEND EXT_LIBRARY ${EXT_LIRBRARY})
else()
    # Single CMake project case (i.e. 'catkin_make'):
    # - The target has not been built when its dependencies require it
    # - Specify target name only
    list(APPEND EXT_LIBRARY data_transmission)
endif()

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES roscpp
#  CATKIN_DEPENDS test
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(/usr/local/include/data_transmission)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/main.cpp)

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
 )

target_link_libraries(${PROJECT_NAME}_node
        ${EXT_LIBRARY}
  )


add_executable(${PROJECT_NAME}_boost_node src/boost_main.cpp)

target_link_libraries(${PROJECT_NAME}_boost_node
        ${catkin_LIBRARIES}
        )

target_link_libraries(${PROJECT_NAME}_boost_node
        ${EXT_BOOST_LIBRARY}
        )

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/roscpp.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})