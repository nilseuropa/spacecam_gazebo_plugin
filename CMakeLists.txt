cmake_minimum_required(VERSION 2.8.3)
project(spacecam_gazebo)

find_package(catkin REQUIRED COMPONENTS gazebo_plugins)
include_directories(include ${catkin_INCLUDE_DIRS})

## Find gazebo
if(POLICY CMP0054)
  cmake_policy(SET CMP0054 NEW)
endif()
find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

## Find OGRE and OGRE-Paging (required for CameraPlugin.hh in Gazebo 5)
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(OGRE OGRE)
  pkg_check_modules(OGRE-Paging OGRE-Paging)
endif()
include_directories(${OGRE_INCLUDE_DIRS} ${OGRE-Paging_INCLUDE_DIRS})
link_directories(${OGRE_LIBRARY_DIRS} ${OGRE-Paging_LIBRARY_DIRS})

## Find Boost
find_package(Boost REQUIRED COMPONENTS thread)
include_directories(${Boost_INCLUDE_DIRS})

catkin_package(
    CATKIN_DEPENDS gazebo_plugins
    INCLUDE_DIRS include
    LIBRARIES
)

## Build
add_library(spacecam_gazebo src/spacecam_gazebo_plugin.cpp)
target_link_libraries(spacecam_gazebo ${GAZEBO_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} CameraPlugin)

## Install

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(TARGETS
  spacecam_gazebo
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
