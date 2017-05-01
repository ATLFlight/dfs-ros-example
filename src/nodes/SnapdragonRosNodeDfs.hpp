/****************************************************************************
 *   Copyright (c) 2017 Sarah Gibson. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name ATLFlight nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <iostream>
#include <signal.h>
#include <stdbool.h>
#include <syslog.h>
#include <unistd.h>
#include "SnapdragonCameraManager.hpp"
#include "SnapdragonDfsManager.hpp"
#include "SnapdragonRosStereoCamParams.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include "tinyxml2.h"

#include <time.h>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <string.h>

namespace Snapdragon {
  class RosNodeDfs;
}

class Snapdragon::RosNodeDfs
{
public:

  RosNodeDfs( ros::NodeHandle nh );

  ~RosNodeDfs();

  void ReadRosParams();
       
  void PrintRosParams();

  int32_t InitCam();

  int32_t Initialize();
  
  void SpinOnce();
  
  void Shutdown();

private:

  //class methods
  
  std::vector<std::string> SplitString( const std::string& input );
    
  void ParseCameraParams ( const tinyxml2::XMLElement* camera_config_element, mvCameraConfiguration& camera_param_values );

  void Configure( const std::string& cfg_file, mvStereoConfiguration& config );

  // data members
  
  ros::NodeHandle nh_;
  std_msgs::Header header_; 
  sensor_msgs::CameraInfo depth_info_;
  sensor_msgs::Image image_l_;  
  sensor_msgs::Image image_r_;  
  sensor_msgs::CameraInfo info_l_;
  sensor_msgs::CameraInfo info_r_;
  tf2_ros::TransformBroadcaster tf_pub_;
  geometry_msgs::TransformStamped transform_stereo_;
  stereo_msgs::DisparityImagePtr disp_msg_;
  sensor_msgs::ImagePtr depth_msg_;

  ros::Publisher pub_image_l_;
  ros::Publisher pub_info_l_;
  ros::Publisher pub_image_r_;
  ros::Publisher pub_info_r_;
  ros::Publisher pub_disparity_;
  ros::Publisher pub_depth_;
  ros::Publisher pub_depth_info_;
  
  int32_t height_, width_;
  Snapdragon::CameraManager* camera_manager_;
  Snapdragon::DfsManager* dfs_manager_;
  Snapdragon::RosStereoCamParams ros_params_;

};
