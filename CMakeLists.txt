#****************************************************************************
# *   Copyright (c) 2017 Sarah Gibson. All rights reserved.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions
# * are met:
# *
# * 1. Redistributions of source code must retain the above copyright
# *  notice, this list of conditions and the following disclaimer.
# * 2. Redistributions in binary form must reproduce the above copyright
# *  notice, this list of conditions and the following disclaimer in
# *  the documentation and/or other materials provided with the
# *  distribution.
# * 3. Neither the name ATLFlight nor the names of its contributors may be
# *  used to endorse or promote products derived from this software
# *  without specific prior written permission.
# *
# * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
# * THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
# * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
# * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
# *
# ****************************************************************************/

cmake_minimum_required(VERSION 2.8.3)
project(snap_dfs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -v -pthread")

if(NOT DEFINED mv_INCLUDE_DIRS)
    set(mv_INCLUDE_DIRS "/usr/include")
endif()
if(NOT DEFINED mv_LINK_DIRS)
    set(mv_LINK_DIRS "/usr/lib")
endif()
message(STATUS "mv_INCLUDE_DIRS: ${mv_INCLUDE_DIRS}")
message(STATUS "mv_LINK_DIRS: ${mv_LINK_DIRS}")

link_directories(
    ${mv_LINK_DIRS}
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES snap_dfs
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)
include_directories(
   ${catkin_INCLUDE_DIRS}
   ${mv_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(snap_dfs_lib
  src/camera/SnapdragonCameraManager.cpp
  src/dfs/SnapdragonDfsManager.cpp
)

## Declare a C++ executable
add_executable(snap_dfs_node src/nodes/SnapdragonRosNodeDfs_main.cpp src/nodes/SnapdragonRosNodeDfs.cpp)

target_include_directories(snap_dfs_lib PUBLIC 
  src/camera
  src/dfs
  ${mv_INCLUDE_DIRS}/mv/
)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes

## Specify libraries to link a library or executable target against
target_link_libraries(snap_dfs_node
  snap_dfs_lib
  mv1
  camera
  tinyxml2
  ${catkin_LIBRARIES}
)

target_include_directories( snap_dfs_node PRIVATE src/nodes/ )

add_custom_command(
TARGET snap_dfs_node
COMMAND echo "Setting the UID bit for the node to run with root privileges"
COMMAND sudo chown root ${CATKIN_DEVEL_PREFIX}/lib/snap_dfs/snap_dfs_node
COMMAND sudo chmod +s ${CATKIN_DEVEL_PREFIX}/lib/snap_dfs/snap_dfs_node
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS snap_dfs_node snap_dfs_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
