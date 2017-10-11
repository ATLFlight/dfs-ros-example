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

#include "SnapdragonDfsRos.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>

Snapdragon::DfsRosNode::DfsRosNode(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
}

Snapdragon::DfsRosNode::~DfsRosNode()
{
  Shutdown();
}

void Snapdragon::DfsRosNode::ReadRosParams()
{  
  pnh_.param<bool>("use_gpu", ros_params_.use_gpu, 0);
  pnh_.param("max_disparity", ros_params_.max_disparity, 32);
  pnh_.param("min_disparity", ros_params_.min_disparity, 0);
  pnh_.param("frame_crop", ros_params_.crop, 21);
  pnh_.param<std::string>("camera_frame_id", ros_params_.frame_id, "camera_link");
}

void Snapdragon::DfsRosNode::PrintRosParams()
{  
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: Use GPU? = " << ros_params_.use_gpu);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: max disparity = " << ros_params_.max_disparity);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: min disparity = " << ros_params_.min_disparity);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: frame crop = " << ros_params_.crop);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: camera frame id = " << ros_params_.frame_id);
}

void Snapdragon::DfsRosNode::PrintMvStereoConfig(mvStereoConfiguration config)
{  
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, left, width x height: " << config.camera[0].pixelWidth << " x " << config.camera[0].pixelHeight);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, left, principal point: [ " << config.camera[0].principalPoint[0] << ", " << config.camera[0].principalPoint[1] << " ]");
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, left, focal length: [ " << config.camera[0].focalLength[0] << ", " << config.camera[0].focalLength[1] << " ]");
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, left, distortion model: " << config.camera[0].distortionModel);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, left, distortion coefficients: [ " << 
    config.camera[0].distortion[0] << " " << 
    config.camera[0].distortion[1] << " " << 
    config.camera[0].distortion[2] << " " << 
    config.camera[0].distortion[3] << " " << 
    config.camera[0].distortion[4] << " " << 
    config.camera[0].distortion[5] << " " << 
    config.camera[0].distortion[6] << " " << 
    config.camera[0].distortion[7] << "]");
    
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, right, width x height: " << config.camera[1].pixelWidth << " x " << config.camera[1].pixelHeight);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, right, principal points: [ " << config.camera[1].principalPoint[0] << ", " << config.camera[1].principalPoint[1] << " ]");
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, right, focal length: [ " << config.camera[1].focalLength[0] << ", " << config.camera[1].focalLength[1] << " ]");
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, right, distortion model: " << config.camera[1].distortionModel);
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, right, distortion coefficients: [ " << 
    config.camera[1].distortion[0] << " " << 
    config.camera[1].distortion[1] << " " << 
    config.camera[1].distortion[2] << " " << 
    config.camera[1].distortion[3] << " " << 
    config.camera[1].distortion[4] << " " << 
    config.camera[1].distortion[5] << " " << 
    config.camera[1].distortion[6] << " " << 
    config.camera[1].distortion[7] << "]");
  
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, extrinsic, translation: [ " << config.translation[0] << ", " << config.translation[1] << ", " << config.translation[2] << " ]");
  ROS_INFO_STREAM("Snapdragon::DfsRosNode: mvStereoConfig, extrinsic, rotation: [ " << config.rotation[0] << ", " << config.rotation[1] << ", " << config.rotation[2] << " ]");
}

int32_t Snapdragon::DfsRosNode::InitDfs(const sensor_msgs::CameraInfoConstPtr& cam_info_l, const sensor_msgs::CameraInfoConstPtr& cam_info_r)
{
  
  // initialize mv stereo config
  mvStereoConfiguration mv_stereo_config;
  memset( &mv_stereo_config, 0, sizeof( mv_stereo_config ) ); // important!

  // left camera parameters
  mv_stereo_config.camera[0].pixelWidth = cam_info_l->width;
  mv_stereo_config.camera[0].pixelHeight = cam_info_l->height;
  mv_stereo_config.camera[0].principalPoint[0] = cam_info_l->K[2];
  mv_stereo_config.camera[0].principalPoint[1] = cam_info_l->K[5];
  mv_stereo_config.camera[0].focalLength[0] = cam_info_l->K[0];
  mv_stereo_config.camera[0].focalLength[1] = cam_info_l->K[4];
  int32_t num_distortion_params_l = 0;
  for (int32_t i=0; i<cam_info_l->D.size(); i++) {
    if (cam_info_l->D[i]) {
      num_distortion_params_l++;
      mv_stereo_config.camera[0].distortion[i] = cam_info_l->D[i];
    }
  }
  mv_stereo_config.camera[0].distortionModel = num_distortion_params_l;

 
  // right camera parameters
  mv_stereo_config.camera[1].pixelWidth = cam_info_r->width;
  mv_stereo_config.camera[1].pixelHeight = cam_info_r->height;
  mv_stereo_config.camera[1].principalPoint[0] = cam_info_r->K[2];
  mv_stereo_config.camera[1].principalPoint[1] = cam_info_r->K[5];
  mv_stereo_config.camera[1].focalLength[0] = cam_info_r->K[0];
  mv_stereo_config.camera[1].focalLength[1] = cam_info_r->K[4];
  int32_t num_distortion_params_r = 0;
  for (int32_t i=0; i<cam_info_l->D.size(); i++) {
    if (cam_info_r->D[i]) {
      num_distortion_params_r++;
      mv_stereo_config.camera[1].distortion[i] = cam_info_r->D[i];
    }
  }
  mv_stereo_config.camera[1].distortionModel = num_distortion_params_r;

  mv_stereo_config.camera[0].memoryStride = mv_stereo_config.camera[0].pixelWidth;
  mv_stereo_config.camera[1].memoryStride = mv_stereo_config.camera[1].pixelWidth;
  
  // extrinsic parameters
  tf2::Matrix3x3 R_r(cam_info_r->R[0],cam_info_r->R[1],cam_info_r->R[2],cam_info_r->R[3],cam_info_r->R[4],cam_info_r->R[5],cam_info_r->R[6],cam_info_r->R[7],cam_info_r->R[8]);
  tf2::Matrix3x3 K_prime(cam_info_r->P[0],cam_info_r->P[1],cam_info_r->P[2],cam_info_r->P[4],cam_info_r->P[5],cam_info_r->P[6],cam_info_r->P[8],cam_info_r->P[9],cam_info_r->P[10]);
  tf2::Vector3 T_prime(cam_info_r->P[3],cam_info_r->P[7],cam_info_r->P[11]);
  tf2::Vector3 T = K_prime.inverse()*T_prime;
  tf2::Vector3 translation = R_r.transpose()*T;
  for (int32_t i=0; i<3; i++) {
    mv_stereo_config.translation[i] = translation[i];
  }
  
  tf2::Matrix3x3 R_l(cam_info_l->R[0],cam_info_l->R[1],cam_info_l->R[2],cam_info_l->R[3],cam_info_l->R[4],cam_info_l->R[5],cam_info_l->R[6],cam_info_l->R[7],cam_info_l->R[8]);
  tf2::Matrix3x3 rotation_axis_angle = R_r.transpose()*R_l;
  
  // convert to scaled axis angle format for mvlib
  float32_t angle = acos(( rotation_axis_angle[0][0] + rotation_axis_angle[1][1] + rotation_axis_angle[2][2] - 1.0)/2.0);
  
  if (angle==0) // if no rotation (may be the case for simulation)
    for (int32_t i=0; i<3; i++) {
      mv_stereo_config.rotation[i] = 0.0;
    }
  else {
    float32_t x = (rotation_axis_angle[2][1] - rotation_axis_angle[1][2])/sqrt(pow((rotation_axis_angle[2][1] - rotation_axis_angle[1][2]),2)
                                   +pow((rotation_axis_angle[0][2] - rotation_axis_angle[2][0]),2)
                                   +pow((rotation_axis_angle[1][0] - rotation_axis_angle[0][1]),2));
    float32_t y = (rotation_axis_angle[0][2] - rotation_axis_angle[2][0])/sqrt(pow((rotation_axis_angle[2][1] - rotation_axis_angle[1][2]),2)
                                   +pow((rotation_axis_angle[0][2] - rotation_axis_angle[2][0]),2)
                                   +pow((rotation_axis_angle[1][0] - rotation_axis_angle[0][1]),2));
    float32_t z = (rotation_axis_angle[1][0] - rotation_axis_angle[0][1])/sqrt(pow((rotation_axis_angle[2][1] - rotation_axis_angle[1][2]),2)
                                   +pow((rotation_axis_angle[0][2] - rotation_axis_angle[2][0]),2)
                                   +pow((rotation_axis_angle[1][0] - rotation_axis_angle[0][1]),2));
    tf2::Vector3 rotation_scaled_axis_angle(x*angle, y*angle, z*angle);

    for (int32_t i=0; i<3; i++) {
      mv_stereo_config.rotation[i] = rotation_scaled_axis_angle[i];
    }
  }

  PrintMvStereoConfig(mv_stereo_config);
 
  // configure DFS cam module
  Snapdragon::DfsManager::DfsCamConfiguration dfs_cam_config;
  memset( &dfs_cam_config, 0, sizeof( dfs_cam_config ) ); //important!
  dfs_cam_config.stereo_config = mv_stereo_config;
  if (ros_params_.use_gpu)  // ROS param
    dfs_cam_config.dfs_mode  = MVDFS_MODE_ALG1_GPU;
  else
    dfs_cam_config.dfs_mode  = MVDFS_MODE_ALG0_CPU;
  dfs_cam_config.max_disparity = ros_params_.max_disparity; // ROS param
  dfs_cam_config.min_disparity = ros_params_.min_disparity; // ROS param
  
  height_ = mv_stereo_config.camera[0].pixelHeight;
  width_ = mv_stereo_config.camera[0].pixelWidth; 
  
  // initialize dfs_manager
  dfs_manager_ = new Snapdragon::DfsManager();
  if (!dfs_manager_->Init(&dfs_cam_config, height_, width_)) {
    ROS_ERROR_STREAM("Snapdragon::DfsRosNode: Error - DFS manager init not successful! ");
    return -1;
  } 
  
  // initialize ROS objects
  header_.stamp = ros::Time::now();
  header_.seq = 0;
  header_.frame_id = ros_params_.frame_id;
  
  depth_info_.height = height_;
  depth_info_.width = width_;
  depth_info_.K = boost::array<double,9> {dfs_manager_->GetDepthCamera().focalLength[0], 0, dfs_manager_->GetDepthCamera().principalPoint[0], 0, dfs_manager_->GetDepthCamera().focalLength[1], dfs_manager_->GetDepthCamera().principalPoint[1], 0, 0, 1};
  
  // set up transform_stereo
  float32_t L[9];
  float32_t R[9];
  mvDFS_GetRectifyingRotations(dfs_manager_->mv_dfs_ptr_, &L[0], &R[0]);
  tf2::Matrix3x3 RR(L[0],L[1],L[2],L[3],L[4],L[5],L[6],L[7],L[8]);
  
  tf2::Quaternion q;
  RR.getRotation(q);

  transform_stereo_.transform.translation.x = 0;
  transform_stereo_.transform.translation.y = 0;
  transform_stereo_.transform.translation.z = 0;
  
  tf2::Quaternion rotation = q.inverse();
  rotation.normalize();

  transform_stereo_.transform.rotation.x = rotation.getX();
  transform_stereo_.transform.rotation.y = rotation.getY();
  transform_stereo_.transform.rotation.z = rotation.getZ();
  transform_stereo_.transform.rotation.w = rotation.getW();

  transform_stereo_.header.frame_id = header_.frame_id;
  transform_stereo_.child_frame_id = "depth_link";

  // Allocate new disparity image message
  disp_msg_ = boost::make_shared<stereo_msgs::DisparityImage>();
  disp_msg_->header = header_;
  disp_msg_->image.header = header_;
  disp_msg_->image.height = height_;
  disp_msg_->image.width = width_; 
  disp_msg_->image.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  disp_msg_->image.step = disp_msg_->image.width * sizeof(float32_t);
  disp_msg_->image.data.resize(disp_msg_->image.height * disp_msg_->image.step);
  disp_msg_->min_disparity=dfs_manager_->GetDfsCamConfig().min_disparity;
  disp_msg_->max_disparity=dfs_manager_->GetDfsCamConfig().max_disparity;
  
  // Allocate new depth image message
  depth_msg_ = boost::make_shared<sensor_msgs::Image>();
  depth_msg_->header = header_;
  depth_msg_->height =  height_;
  depth_msg_->width =  width_;
  depth_msg_->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  depth_msg_->step = depth_msg_->width * sizeof(float32_t);
  depth_msg_->data.resize(depth_msg_->height * depth_msg_->step);

  // done initializing
  initialized_ = true;
  return 0;
}

int32_t Snapdragon::DfsRosNode::Initialize()
{
  // not initialized yet
  initialized_ = false;
  
  // fetch DFS params from ROS parameter server
  ReadRosParams(); 
  PrintRosParams();
  
  pub_disparity_ = nh_.advertise<stereo_msgs::DisparityImage>("dfs/disparity_image",10);
  pub_depth_image_ = nh_.advertise<sensor_msgs::Image>("dfs/depth/image_raw",10);
  pub_depth_info_ = nh_.advertise<sensor_msgs::CameraInfo>("dfs/depth/camera_info",10);
  pub_point_cloud_ = nh_.advertise<sensor_msgs::PointCloud>("dfs/point_cloud",10);

  // set up ROS subscribers
  image_sub_l_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "left/image_raw", 1);
  info_sub_l_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "left/camera_info", 1);
  image_sub_r_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "right/image_raw", 1);
  info_sub_r_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "right/camera_info", 1);
  image_sub_d_ = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "dfs/depth/image_raw", 1);
  info_sub_d_ = new message_filters::Subscriber<sensor_msgs::CameraInfo>(nh_, "dfs/depth/camera_info", 1);

  // register image callbacks
  sync_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo>(*image_sub_l_, *info_sub_l_, *image_sub_r_, *info_sub_r_, 10);
  sync_->registerCallback(boost::bind(&Snapdragon::DfsRosNode::CameraCallback, this, _1, _2, _3, _4));
  sync_depth_ = new message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo>(*image_sub_d_, *info_sub_d_, 10);
  sync_depth_->registerCallback(boost::bind(&Snapdragon::DfsRosNode::DepthCallback, this, _1, _2));
 
  return 0;
}

void Snapdragon::DfsRosNode::CameraCallback(const sensor_msgs::ImageConstPtr& image_l,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_l,
                    const sensor_msgs::ImageConstPtr& image_r,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_r)
{
  if (!initialized_) {
    try {
      InitDfs(cam_info_l, cam_info_r);
    }
    catch (std::runtime_error& e) {
      ROS_ERROR_STREAM("Snapdragon::DfsRosNode: Error initializing dfs!");
    }
  }

  // call to MV
  dfs_manager_->Process(image_l->data.data(), image_r->data.data());
  
  // update ROS headers
  header_.stamp = ros::Time::now();
  header_.seq++;

  // create output disparity image
  disp_msg_->image.header = header_;
  float32_t * disp_image =  reinterpret_cast<float32_t*>(&disp_msg_->image.data[0]);
  uint16_t* cur_disparity = dfs_manager_->GetCurDisparity();
  for(int32_t i = ros_params_.crop; i < height_-ros_params_.crop; i++)
    for(int32_t j = ros_params_.crop; j < width_-ros_params_.crop; j++)
      disp_image[i*width_+j] = cur_disparity[i*width_+j];

  pub_disparity_.publish(disp_msg_);

  // create output depth image and point cloud
  depth_msg_->header = header_;
  float32_t * depth_image =  reinterpret_cast<float32_t*>(&depth_msg_->data[0]);
  float32_t* cur_inv_depth = dfs_manager_->GetCurInvDepth();
  for(int32_t i = ros_params_.crop; i < height_-ros_params_.crop; i++)
    for(int32_t j = ros_params_.crop; j < width_-ros_params_.crop; j++) {
      float32_t d = cur_inv_depth[i*width_+j];
      d = (d>0) ? 1.0/d : 0;
      depth_image[i*width_+j] = d;
    }

  pub_depth_image_.publish(depth_msg_);

  // uncomment the following lines for debug
  //ROS_INFO_STREAM("Snapdragon::DfsRosNode: center pixel disparity = " << disp_image[(height_-ros_params_.crop)/2*width_+(width_-ros_params_.crop)/2]);
  //ROS_INFO_STREAM("Snapdragon::DfsRosNode: center pixel depth = " << depth_image[(height_-ros_params_.crop)/2*width_+(width_-ros_params_.crop)/2]);

  // publish depth camera Camera Info
  depth_info_.header = header_;
  pub_depth_info_.publish(depth_info_);

  // publish depth camera transform
  transform_stereo_.header.stamp = header_.stamp;
  tf_pub_.sendTransform(transform_stereo_);
  
}

void Snapdragon::DfsRosNode::DepthCallback(const sensor_msgs::ImageConstPtr& image_d,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_d)
{
  float fx = cam_info_d->K[0];
  float fy = cam_info_d->K[2];
  float cx = cam_info_d->K[4];
  float cy = cam_info_d->K[5];
  
  sensor_msgs::PointCloud point_cloud_msg;
  geometry_msgs::Point32 dfs_point;
  point_cloud_msg.header = header_;

  float* depth_array = (float*) (&(image_d->data[0]));
  for(int32_t i = ros_params_.crop; i < height_-ros_params_.crop; i++)
    for(int32_t j = ros_params_.crop; j < width_-ros_params_.crop; j++) {
      float32_t d = depth_array[i*width_+j];
      dfs_point.x = d*(j - cx) / fx;
      dfs_point.y = d*(i - cy) / fy;
      dfs_point.z = d;
      point_cloud_msg.points.push_back(dfs_point);
    }

  pub_point_cloud_.publish(point_cloud_msg);

}

void Snapdragon::DfsRosNode::Shutdown()
{
  dfs_manager_->Deinit();
  delete dfs_manager_;
}
