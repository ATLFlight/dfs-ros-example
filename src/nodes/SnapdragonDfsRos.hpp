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
#include <fstream>
#include <signal.h>
#include <stdbool.h>
#include <syslog.h>
#include <unistd.h>
#include "SnapdragonDfsManager.hpp"
#include "SnapdragonDfsRosParams.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <stereo_msgs/DisparityImage.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point32.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>

#include <time.h>
#include <boost/filesystem.hpp>
#include <stdexcept>
#include <string.h>

namespace Snapdragon {
  class DfsRosNode;
}

class Snapdragon::DfsRosNode
{
public:

  DfsRosNode( ros::NodeHandle nh, ros::NodeHandle pnh );

  ~DfsRosNode();

  void ReadRosParams();
       
  void PrintRosParams();
  
  void PrintMvStereoConfig(mvStereoConfiguration config);
  
  int32_t Initialize();
  
  int32_t InitDfs(const sensor_msgs::CameraInfoConstPtr& cam_info_l,
                  const sensor_msgs::CameraInfoConstPtr& cam_info_r);

  void CameraCallback(const sensor_msgs::ImageConstPtr& image_l,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_l,
                    const sensor_msgs::ImageConstPtr& image_r,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_r);
  
  void DepthCallback(const sensor_msgs::ImageConstPtr& image_d,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_d);

  void Shutdown();

private:

  bool initialized_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std_msgs::Header header_; 
  stereo_msgs::DisparityImagePtr disp_msg_;
  sensor_msgs::ImagePtr depth_msg_;
  sensor_msgs::CameraInfo depth_info_;
  geometry_msgs::TransformStamped transform_stereo_;
  tf2_ros::TransformBroadcaster tf_pub_;
 
  message_filters::Subscriber<sensor_msgs::Image>* image_sub_l_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* info_sub_l_;
  message_filters::Subscriber<sensor_msgs::Image>* image_sub_r_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* info_sub_r_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo>* sync_;
  message_filters::Subscriber<sensor_msgs::Image>* image_sub_d_;
  message_filters::Subscriber<sensor_msgs::CameraInfo>* info_sub_d_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>* sync_depth_;

  ros::Publisher pub_disparity_;
  ros::Publisher pub_depth_image_;
  ros::Publisher pub_depth_info_;
  ros::Publisher pub_point_cloud_;
  
  int32_t height_, width_;
  Snapdragon::DfsManager* dfs_manager_;
  Snapdragon::DfsRosParams ros_params_;

};
