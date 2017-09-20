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

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2
  tf2_ros
  nodelet
  sensor_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -v -pthread -fPIC")

if(NOT DEFINED MV_INCLUDE)
    set(MV_INCLUDE "/usr/include")
endif()
if(NOT DEFINED MV_LIB)
    set(MV_LIB "/usr/lib")
endif()
message(STATUS "MV_INCLUDE: ${MV_INCLUDE}")
message(STATUS "MV_LIB: ${MV_LIB}")

link_directories(
    ${MV_LIB}
)

catkin_package(
  CATKIN_DEPENDS roscpp nodelet sensor_msgs
  LIBRARIES snap_dfs_nodelet
)

include_directories(
   ${catkin_INCLUDE_DIRS}
   ${MV_INCLUDE}
)

add_library(snap_dfs
  src/dfs/SnapdragonDfsManager.cpp
)
target_include_directories(snap_dfs PUBLIC
  src/dfs
  ${MV_INCLUDE}/mv/
)
target_link_libraries(snap_dfs
  mv1
)

add_library(snap_dfs_nodelet src/nodes/SnapdragonDfsRos_nodelet.cpp src/nodes/SnapdragonDfsRos.cpp)
target_link_libraries(snap_dfs_nodelet
  snap_dfs
  ${catkin_LIBRARIES}
)
target_include_directories( snap_dfs_nodelet PRIVATE src/nodes/ )

add_executable(snap_dfs_node src/nodes/SnapdragonDfsRos_node.cpp src/nodes/SnapdragonDfsRos.cpp)
target_link_libraries(snap_dfs_node
  snap_dfs
  ${catkin_LIBRARIES}
)
target_include_directories( snap_dfs_node PRIVATE src/nodes/ )

add_custom_command(
TARGET snap_dfs_node
COMMAND echo "Setting the UID bit for the node to run with root privileges"
COMMAND sudo chown root ${CATKIN_DEVEL_PREFIX}/lib/snap_dfs/snap_dfs_node
COMMAND sudo chmod +s ${CATKIN_DEVEL_PREFIX}/lib/snap_dfs/snap_dfs_node
)

install(TARGETS snap_dfs_node snap_dfs snap_dfs_nodelet
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
install(FILES snap_dfs_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
