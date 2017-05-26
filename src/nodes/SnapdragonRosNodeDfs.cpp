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

#include "SnapdragonRosNodeDfs.hpp"

Snapdragon::RosNodeDfs::RosNodeDfs(ros::NodeHandle nh) : nh_(nh)
{
}

Snapdragon::RosNodeDfs::~RosNodeDfs()
{
  Shutdown();
}

std::vector<std::string> Snapdragon::RosNodeDfs::SplitString( const std::string& input )
{
   std::vector<std::string> output;
   size_t current;
   size_t next = -1;
   do
   {
    current = next + 1;
    next = input.find_first_of( " ", current );
    if ( input.substr(current, next - current).empty() )
    {
     continue;
    }
    output.push_back( input.substr(current, next - current) );
   }
   while(  next != std::string::npos );

   return output;
}

void Snapdragon::RosNodeDfs::ParseCameraParams ( const tinyxml2::XMLElement* camera_config_element, mvCameraConfiguration& camera_param_values )
{
   // Get the image size
   const char* sizes = camera_config_element->Attribute( "size" );
   if ( sizes == nullptr )
   {
    throw std::invalid_argument( "Bad config: Missing size attribute" );
   }

   std::vector<std::string> values = SplitString( sizes );
   if ( values.size() != 2 )
   {
    throw std::invalid_argument( "Bad config: Missing size value" );
   }

   // Fill in the configuration - Camera
   camera_param_values.pixelWidth = atoi(values[0].c_str());
   camera_param_values.pixelHeight = atoi(values[1].c_str());

   const char* principal_points = camera_config_element->Attribute( "principal_point" );
   if ( principal_points == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad config: Missing principal_point32_t attribute"
    );
   }
   values = SplitString( principal_points );
   if ( values.size() != 2 )
   {
    throw std::invalid_argument
    (
     "Bad config: Missing principal_point32_t value"
    );
   }

   camera_param_values.principalPoint[0] = atof(values[0].c_str());
   camera_param_values.principalPoint[1] = atof(values[1].c_str());

   const char* focal_lengths = camera_config_element->Attribute( "focal_length" );
   if ( focal_lengths == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad config: Missing focal_length attribute"
    );
   }
   values = SplitString( focal_lengths );
   if ( values.size() != 2 )
   {
    throw std::invalid_argument
    (
     "Bad config: Missing focal_length value"
    );
   }

   camera_param_values.focalLength[0] = atof(values[0].c_str());
   camera_param_values.focalLength[1] = atof(values[1].c_str());
   
   const char* fisheye = camera_config_element->Attribute( "fishEye" );
   ROS_INFO_STREAM("Snapdragon::RosNodeDfs: fisheye = " << fisheye);
   if ( fisheye == nullptr )
   {
    camera_param_values.distortionModel = 4;
   }
   else {
     if (strcmp( fisheye, "true") == 0 ) {
      camera_param_values.distortionModel = 10;
     }
     else {
      camera_param_values.distortionModel = 4;
     }
   }
   ROS_INFO_STREAM("Snapdragon::RosNodeDfs: 1 of 2: distortionModel = " << camera_param_values.distortionModel);
   const char* distortion = camera_config_element->Attribute( "radial_distortion" );
   if ( distortion == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad config: Missing radial_distortion attribute"
    );
   }
   values = SplitString( distortion );
   int32_t i = 0;
   for ( auto next : values )
   {
    camera_param_values.distortion[i] = atof(next.c_str());
    i++;
   }

   int32_t num_distortion_params = 0;
   for (int32_t n : camera_param_values.distortion)
    if (camera_param_values.distortion[n])
      num_distortion_params++;

   if (!fisheye)
    camera_param_values.distortionModel = num_distortion_params;
   
   ROS_INFO_STREAM("Snapdragon::RosNodeDfs: 2 of 2: distortionModel = " << camera_param_values.distortionModel);
}

void Snapdragon::RosNodeDfs::Configure( const std::string& cfg_file, mvStereoConfiguration& config )
{
   // Open the XML file
   tinyxml2::XMLDocument cfg_doc;
   cfg_doc.PrintError();
   cfg_doc.LoadFile( cfg_file.c_str() );

   tinyxml2::XMLHandle doc_handle( &cfg_doc );

   // Get the Left camera calibration Node
   tinyxml2::XMLElement* calib_node =
    doc_handle.FirstChildElement( "Configuration" ).
    FirstChildElement( "DeviceProfile" ).
     FirstChildElement( "StereoRig" ).
      FirstChildElement( "CameraLeft" ).
         FirstChildElement( "Calibration" ).ToElement();
   if ( calib_node == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad configuration file. Missing CameraLeft calibration data."
    );
   }
   ParseCameraParams( calib_node, config.camera[0] );
   
   // Get the Right camera calibration Node
   calib_node =
    doc_handle.FirstChildElement( "Configuration" ).
    FirstChildElement( "DeviceProfile" ).
     FirstChildElement( "StereoRig" ).
      FirstChildElement( "CameraRight" ).
         FirstChildElement( "Calibration" ).ToElement();
   if ( calib_node == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad configuration file. Missing CameraRight calibration data."
    );
   }
   ParseCameraParams( calib_node, config.camera[1] );

   // Get the Pose info
   tinyxml2::XMLElement* pose_element =
    doc_handle.FirstChildElement( "Configuration" ).
    FirstChildElement( "DeviceProfile" ).
     FirstChildElement( "StereoRig" ).
      FirstChildElement( "Pose" ).ToElement();

   if ( pose_element == nullptr )
   {
    throw std::invalid_argument
    (
     "Bad configuration file. Missing Pose data."
    );
   }
  
   const char* translation = pose_element->Attribute( "translation" );
   if ( translation == nullptr )
   {
    throw std::invalid_argument( "Bad config: Missing translation attribute" );
   }

   std::vector<std::string> values = SplitString( translation );
   int32_t i = 0;
   for ( auto next : values )
   {
    config.translation[i] = atof(next.c_str());
    i++;
   }

   const char* rotation = pose_element->Attribute( "rotation" );
   if ( rotation == nullptr )
   {
    throw std::invalid_argument( "Bad config: Missing translation rotation" );
   }

   values = SplitString( rotation );
   i = 0;
   for ( auto next : values )
   {
    config.rotation[i] = atof(next.c_str());
    i++;
   }
}

void Snapdragon::RosNodeDfs::ReadRosParams()
{  
  ros::param::param<std::string>("~config_filename", ros_params_.filename, "");
  ros::param::param("~frame_rate", ros_params_.frame_rate, 30);
  ros::param::param<float>("~exposure_time", ros_params_.exposure_time, 0.16);
  ros::param::param<float>("~gain", ros_params_.gain, 0.6);
  ros::param::param<bool>("~use_gpu", ros_params_.use_gpu, 0);
  ros::param::param("~max_disparity", ros_params_.max_disparity, 32);
  ros::param::param("~min_disparity", ros_params_.min_disparity, 0);
  ros::param::param("~frame_crop", ros_params_.chop, 21);
  ros::param::param<std::string>("~camera_frame_id", ros_params_.frame_id, "camera_link");
}

void Snapdragon::RosNodeDfs::PrintRosParams()
{  
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: config filename = " << ros_params_.filename);
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: frame_rate = " << ros_params_.frame_rate);
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: Stereo camera exposure time = " << ros_params_.exposure_time); 
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: Stereo camera gain = " << ros_params_.gain); 
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: Use GPU? = " << ros_params_.use_gpu);
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: max disparity = " << ros_params_.max_disparity); 
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: min disparity = " << ros_params_.min_disparity);
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: frame crop = " << ros_params_.chop); 
  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: camera frame id = " << ros_params_.frame_id);
}

int32_t Snapdragon::RosNodeDfs::InitCam()
{
  
  // initialize camera manager and mv stereo config
  camera_manager_ = new Snapdragon::CameraManager();
  mvStereoConfiguration   mv_stereo_config;
  memset( &mv_stereo_config, 0, sizeof( mv_stereo_config ) ); // important!

  // read in stereo calibration file
  Configure(ros_params_.filename, mv_stereo_config);
  std::ifstream src(ros_params_.filename, std::ios::binary);
  std::ofstream dest("./Configuration.Stereo.xml", std::ios::binary);
  dest << src.rdbuf();

  // configure cameras
  Snapdragon::CameraManagerConfig cameraconfig;
  cameraconfig.p_size_[0] = mv_stereo_config.camera[0].pixelWidth;
  cameraconfig.p_size_[1] = mv_stereo_config.camera[0].pixelHeight;
  cameraconfig.p_stride_ = mv_stereo_config.camera[0].pixelWidth;
  if (mv_stereo_config.camera[0].pixelWidth==640)
    cameraconfig.resolution_ = Snapdragon::CameraManagerConfig::CaptureResolution::CAPTURE_RESOLUTION_VGA_640_480;
  else if (mv_stereo_config.camera[0].pixelWidth==320)
    cameraconfig.resolution_ = Snapdragon::CameraManagerConfig::CaptureResolution::CAPTURE_RESOLUTION_QVGA_320_240;
  else {
    ROS_ERROR_STREAM("Snapdragon::RosNodeDfs: Error - Invalid resolution in config file! ");
    return -1;
  } 

  cameraconfig.frame_rate_ = ros_params_.frame_rate; // ROS param
  cameraconfig.capture_mode_ = Snapdragon::CameraManagerConfig::CAPTURE_MODE_PREVIEW;
  cameraconfig.func_ = Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO; 
  cameraconfig.exposure_ = ros_params_.exposure_time; // ROS param
  cameraconfig.gain_ = ros_params_.gain; // ROS param
  cameraconfig.output_format_ = Snapdragon::CameraManagerConfig::OUTPUT_FORMAT_NV12;
   
  // configure DFS cam module
  Snapdragon::DfsManager::DfsCamConfiguration dfs_cam_config;
  memset( &dfs_cam_config, 0, sizeof( dfs_cam_config ) ); //important!
  dfs_cam_config.stereo_config = mv_stereo_config;
  if (ros_params_.use_gpu)  // ROS param
    dfs_cam_config.dfs_mode  = MV_MODE_GPU;
  else
    dfs_cam_config.dfs_mode  = MV_MODE_SPEED;
  dfs_cam_config.max_disparity = ros_params_.max_disparity; // ROS param
  dfs_cam_config.min_disparity = ros_params_.min_disparity; // ROS param
  dfs_cam_config.camera_manager_image_format = Snapdragon::CameraManagerConfig::CAM_MAN_OUTPUT_FORMAT_8_BIT_GRAY;
  
  // initialize camera and dfs_manager
  if (!camera_manager_->Init( cameraconfig )) {
    ROS_ERROR_STREAM("Snapdragon::RosNodeDfs: Error - Camera init not successful! ");
    return -1;
  } 
  dfs_manager_ = new Snapdragon::DfsManager();
  if (!dfs_manager_->Init(camera_manager_, &dfs_cam_config)) {
    ROS_ERROR_STREAM("Snapdragon::RosNodeDfs: Error - DFS manager init not successful! ");
    return -1;
  } 

  ROS_INFO_STREAM("Snapdragon::RosNodeDfs: Starting stereo cameras");
  if( !camera_manager_->Start() ) {
    ROS_ERROR_STREAM("Snapdragon::RosNodeDfs: Error in starting stereo cameras!");
    return -1;
  } else {
    ROS_INFO_STREAM("Snapdragon::RosNodeDfs: Camera init successful");
  }
  return 0;
}

int32_t Snapdragon::RosNodeDfs::Initialize()
{
  // fetch ROS camera/DFS params from parameter server
  ReadRosParams(); 
  PrintRosParams();
  
  // set up ROS publishers
  pub_image_l_ = nh_.advertise<sensor_msgs::Image>("left/image_raw",10);
  pub_info_l_ = nh_.advertise<sensor_msgs::CameraInfo>("left/camera_info",10);
  pub_image_r_ = nh_.advertise<sensor_msgs::Image>("right/image_raw",10);
  pub_info_r_ = nh_.advertise<sensor_msgs::CameraInfo>("right/camera_info",10);
  pub_disparity_ = nh_.advertise<stereo_msgs::DisparityImage>("mv_disparity",10);
  pub_depth_ = nh_.advertise<sensor_msgs::Image>("mv_depth",10);
  pub_depth_info_ = nh_.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info",10);

  // init MV camera and DFS module
  try {
    InitCam();
  }
  catch (std::runtime_error& e) {
    ROS_ERROR_STREAM("Snapdragon::RosNodeDfs: Error initializing dfs!");
    return 1;
  }
  
  // initialize ROS variables
  height_ =  dfs_manager_->camera_manager_ptr_->GetCameraConfig().p_size_[1]; //number of rows
  width_ =  dfs_manager_->camera_manager_ptr_->GetCameraConfig().p_size_[0]; //number of columns

  header_.stamp = ros::Time::now();
  header_.seq = 0;
  header_.frame_id = ros_params_.frame_id;
  
  image_l_.width = width_;
  image_l_.height = height_;
  image_l_.is_bigendian = 0;
  image_l_.encoding = sensor_msgs::image_encodings::MONO8;
  
  image_r_.width = width_;
  image_r_.height = height_;
  image_r_.is_bigendian = 0;
  image_r_.encoding = sensor_msgs::image_encodings::MONO8;
  
  info_l_.width = width_;
  info_l_.height = height_;
  info_r_.width = width_;
  info_r_.height = height_;
  
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

  return 0;
}

void Snapdragon::RosNodeDfs::SpinOnce()
{
  // call to MV
  dfs_manager_->DfsCamProcessingMain();
  
  unsigned char * img_l;
  unsigned char * img_r;
  img_l = dfs_manager_->GetCurFrameBufferL();
  img_r = dfs_manager_->GetCurFrameBufferR();
  
  // update ROS headers
  header_.stamp = ros::Time::now();
  header_.seq++;

  // publish raw stereo images
  image_l_.header = header_;
  image_l_.step = width_*sizeof(img_l[0]);
  image_l_.data.assign(img_l,img_l+size_t(height_*image_l_.step));

  image_r_.header = header_;
  image_r_.step = width_*sizeof(img_r[0]);
  image_r_.data.assign(img_r,img_r+size_t(height_*image_r_.step));

  info_l_.header = header_;
  info_r_.header = header_;

  pub_image_l_.publish(image_l_);
  pub_image_r_.publish(image_r_);
  pub_info_l_.publish(info_l_);
  pub_info_r_.publish(info_r_);

  // create output disparity image
  disp_msg_->image.header = header_;
  float32_t * disp_image_ =  reinterpret_cast<float32_t*>(&disp_msg_->image.data[0]);
  uint16_t* cur_disparity = dfs_manager_->GetCurDisparity();
  for(int32_t i = ros_params_.chop; i < height_-ros_params_.chop; i++)
    for(int32_t j = ros_params_.chop; j < width_-ros_params_.chop; j++)
      disp_image_[i*width_+j] = cur_disparity[i*width_+j];

  pub_disparity_.publish(disp_msg_);

  // create output depth image
  depth_msg_->header = header_;
  float32_t * depth_image_ =  reinterpret_cast<float32_t*>(&depth_msg_->data[0]);
  float32_t* cur_inv_depth = dfs_manager_->GetCurInvDepth();
  for(int32_t i = ros_params_.chop; i < height_-ros_params_.chop; i++)
    for(int32_t j = ros_params_.chop; j < width_-ros_params_.chop; j++) {
      float32_t foo = cur_inv_depth[i*width_+j];
      depth_image_[i*width_+j] = (foo>0) ? 1.0/foo : 0;
    }

  pub_depth_.publish(depth_msg_);
  
  //ROS_INFO_STREAM("Snapdragon::RosNodeDfs: center pixel disparity = " << disp_image_[(height_-ros_params_.chop)/2*width_+(width_-ros_params_.chop)/2]);
  //ROS_INFO_STREAM("Snapdragon::RosNodeDfs: center pixel depth = " << depth_image_[(height_-ros_params_.chop)/2*width_+(width_-ros_params_.chop)/2]);

  // publish depth camera Camera Info
  depth_info_.header = header_;
  pub_depth_info_.publish(depth_info_);

  // publish depth camera transform
  transform_stereo_.header.stamp = header_.stamp;
  tf_pub_.sendTransform(transform_stereo_);
  
}

void Snapdragon::RosNodeDfs::Shutdown()
{
  dfs_manager_->Deinit();
  camera_manager_->Stop();
  camera_manager_->Deinit();
  delete camera_manager_;
  delete dfs_manager_;
}
