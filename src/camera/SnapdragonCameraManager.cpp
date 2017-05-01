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

#include "SnapdragonCameraManager.hpp"
#include <algorithm>

Snapdragon::CameraManager::CameraManager()
{
  cam_id_ = 0;
  frame_id_ = 0;
  cpa_obj_ = NULL;
  srw_obj_ = NULL;
}

Snapdragon::CameraManager::~CameraManager()
{
}

void Snapdragon::CameraManager::EnableCpaLogging( mvSRW_Writer * writer )
{
  srw_obj_ = writer;
}

void Snapdragon::CameraManager::ApplyCpa(CameraFrameType cur_frame)
{
  float32_t exposure = 0.0f, gain = 0.0f;
  mvCPA_AddFrame( cpa_obj_, cur_frame.frame->data, camera_manager_cfg_.p_size_[0], camera_manager_cfg_.p_size_[1], camera_manager_cfg_.p_stride_ );
  mvCPA_GetValues( cpa_obj_, &exposure, &gain );
  SetExposureAndGain( exposure, gain );
  if( srw_obj_ )
    mvSRW_Writer_AddCameraSettings( srw_obj_, cur_frame.timestamp, gain, exposure, 0 );
}

void Snapdragon::CameraManager::onError()
{
  std::cout << "Snapdragon::CameraManager, ERR, CameraManager onError!"<<std::endl;
}

float Snapdragon::CameraManager::GetCameraExposureTime()
{
  float exposure_value = Snapdragon::CameraManagerConfig::MIN_EXPOSURE_VALUE + camera_exposure_hist_[1] * (Snapdragon::CameraManagerConfig::MAX_EXPOSURE_VALUE - Snapdragon::CameraManagerConfig::MIN_EXPOSURE_VALUE);
  return (camera_manager_cfg_.row_period_us_ * exposure_value);
}

void Snapdragon::CameraManager::SetExposureAndGain( float exposure, float gain )
{
  if( exposure == camera_manager_cfg_.exposure_ && gain == camera_manager_cfg_.gain_ )
  {
    //parameters are the same, don't have to set again
    return;
  }

  int32_t exposure_value; 
  int32_t gain_value;
  bool update_exposure = false; 
  bool update_gain = false;

  if( exposure >= 0.f && exposure <= 1.f )
  {
    camera_manager_cfg_.exposure_ = exposure;
    camera_exposure_hist_[1] = camera_exposure_hist_[0];
    camera_exposure_hist_[0] = exposure;
    exposure_value = Snapdragon::CameraManagerConfig::MIN_EXPOSURE_VALUE + camera_manager_cfg_.exposure_ * 
      (Snapdragon::CameraManagerConfig::MAX_EXPOSURE_VALUE - CameraManagerConfig::MIN_EXPOSURE_VALUE);
    update_exposure = true;
  }
  if( gain >= 0.f && gain <= 1.f )
  {
    camera_manager_cfg_.gain_ = gain;
    gain_value = Snapdragon::CameraManagerConfig::MIN_GAIN_VALUE + camera_manager_cfg_.gain_ * 
      (Snapdragon::CameraManagerConfig::MAX_GAIN_VALUE - Snapdragon::CameraManagerConfig::MIN_GAIN_VALUE);
    update_gain = true;
  }
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW )
  {
    if( update_exposure )
    {
      camera_params_.setManualExposure( exposure_value );
      std::cout << "Snapdragon::CameraManager, INFO, settings exposure  " << exposure_value << std::endl;
    }
     
    if( update_gain )
    {
      camera_params_.setManualGain( gain_value );
      std::cout << "Snapdragon::CameraManager, INFO, settings gain  " << gain_value << std::endl;
    }
     
    camera_stereo_->SetParameters( camera_params_ );

  }
  else
#endif
  {
    if( update_exposure )
    {
      char buffer[33];
      snprintf( buffer, sizeof( buffer ), "%d", exposure_value );
      camera_params_.set( "qc-exposure-manual", buffer );
    }
    if( update_gain )
    {
      char buffer[33];
      snprintf( buffer, sizeof( buffer ), "%d", gain_value );
      camera_params_.set( "qc-gain-manual", buffer );
    }
    int return_code = EXIT_SUCCESS;
    return_code = camera_params_.commit();
    if (return_code)
    {
      std::cout << "Snapdragon::CameraManager, ERR, camera params commit failed!" << std::endl;
    }
  }
  
}

int32_t Snapdragon::CameraManager::SetParameters()
{
  int32_t focus_mode_idx = 3;
  int32_t wb_mode_idx = 2;
  int32_t iso_mode_idx = 5;   /// iso800
  int32_t p_fps_idx = -1;
  int32_t v_fps_idx = -1;
  int32_t default_p_fps = 3; /// 30 fps
  int32_t default_v_fps = 3; /// 30 fps
  camera_caps_.p_sizes_ = camera_params_.getSupportedPreviewSizes();
  camera_caps_.v_sizes_ = camera_params_.getSupportedVideoSizes();
  camera_caps_.focus_modes_ = camera_params_.getSupportedFocusModes();
  camera_caps_.wb_modes_ = camera_params_.getSupportedWhiteBalance();
  camera_caps_.iso_modes_ = camera_params_.getSupportedISO();
  camera_caps_.brightness_ = camera_params_.getSupportedBrightness();
  camera_caps_.sharpness_ = camera_params_.getSupportedSharpness();
  camera_caps_.contrast_ = camera_params_.getSupportedContrast();
  camera_caps_.preview_fps_ranges_ = camera_params_.getSupportedPreviewFpsRanges();
  camera_caps_.video_fps_values_ = camera_params_.getSupportedVideoFps();

  camera_exposure_hist_[0] = camera_manager_cfg_.exposure_;
  camera_exposure_hist_[1] = camera_manager_cfg_.exposure_;

  camera::ImageSize frame_size;
  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO )
    frame_size.width = camera_manager_cfg_.p_size_[0] * 2;
  else
    frame_size.width = camera_manager_cfg_.p_size_[0];

  frame_size.height = camera_manager_cfg_.p_size_[1];

  if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_PREVIEW )
  {
    std::cout << "Snapdragon::CameraManager, INFO, settings preview size " << frame_size.width << " & " << frame_size.height << std::endl;
    camera_params_.setPreviewSize( frame_size );
  }
  else if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_VIDEO )
  {
    std::cout << "Snapdragon::CameraManager, INFO, settings video size " << frame_size.width << " & " << frame_size.height << std::endl;
    camera_params_.setVideoSize( frame_size );
  }

  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_HIRES )
  {
    std::cout << "Snapdragon::CameraManager, INFO, setting ISO mode: " << camera_caps_.iso_modes_[iso_mode_idx].c_str() << std::endl;
    camera_params_.setISO( camera_caps_.iso_modes_[iso_mode_idx] );
    std::cout << "Snapdragon::CameraManager, INFO, setting focus mode: " << camera_caps_.focus_modes_[focus_mode_idx].c_str() << std::endl;
    camera_params_.setFocusMode( camera_caps_.focus_modes_[focus_mode_idx] );
    std::cout << "Snapdragon::CameraManager, INFO, setting WB mode: " << camera_caps_.wb_modes_[wb_mode_idx].c_str() << std::endl;
    camera_params_.setWhiteBalance( camera_caps_.wb_modes_[wb_mode_idx] );
  }
  if( camera_caps_.preview_fps_ranges_.size() <= default_p_fps )
  {
    std::cout << "Snapdragon::CameraManager, INFO, default preview fps index " << default_p_fps <<"  greater than number of supported fps ranges " << camera_caps_.preview_fps_ranges_.size() <<" setting to " << camera_caps_.preview_fps_ranges_.size() - 1 <<" as default\n" << std::endl;
    default_p_fps = camera_caps_.preview_fps_ranges_.size() - 1;
  }
  if( camera_caps_.video_fps_values_.size() <= default_v_fps )
  {
    std::cout << "Snapdragon::CameraManager, INFO, default video fps index " << default_v_fps <<" greater than number of supported fps ranges "<< camera_caps_.video_fps_values_.size() <<" setting to " << camera_caps_.video_fps_values_.size() - 1 << " as default" << std::endl;
    default_v_fps = camera_caps_.video_fps_values_.size() - 1;
  }
  for( int32_t i = 0; i < camera_caps_.preview_fps_ranges_.size(); ++i )
  {
    if( (camera_caps_.preview_fps_ranges_[i].max) / 1000 == camera_manager_cfg_.frame_rate_ )
    {
      p_fps_idx = i;
      break;
    }
  }
  for( int32_t i = 0; i < camera_caps_.video_fps_values_.size(); ++i )
  {
    if( camera_caps_.video_fps_values_[i] == camera_manager_cfg_.frame_rate_ )
    {
      v_fps_idx = i;
      break;
    }
  }
  if( p_fps_idx == -1 )
  {
    std::cout << "Snapdragon::CameraManager, INFO, couldnt find preview fps index for requested framerate "<< camera_manager_cfg_.frame_rate_ <<" setting default index " << default_p_fps << std::endl;
    p_fps_idx = default_p_fps;
  }
  if( v_fps_idx == -1 )
  {
    std::cout << "Snapdragon::CameraManager, INFO, couldnt find video fps index for requested framerate " << camera_manager_cfg_.frame_rate_ <<" setting default index " << default_v_fps << std::endl;
    v_fps_idx = default_v_fps;
  }

  std::cout << "Snapdragon::CameraManager, INFO, setting preview fps range(idx " << p_fps_idx << " ): " << camera_caps_.preview_fps_ranges_[p_fps_idx].min << " , " << camera_caps_.preview_fps_ranges_[p_fps_idx].max << std::endl;
  camera_params_.setPreviewFpsRange( camera_caps_.preview_fps_ranges_[p_fps_idx] );

  if( camera_manager_cfg_.output_format_ == Snapdragon::CameraManagerConfig::OUTPUT_FORMAT_RAW )
  {
    std::cout << "Snapdragon::CameraManager, INFO, setting output_format OUTPUT_FORMAT_RAW" << std::endl;
    camera_params_.set( "preview-format", "bayer-rggb" );
    camera_params_.set( "picture-format", "bayer-mipi-10gbrg" );
    camera_params_.set( "raw-size", "640x480" );
  }
  if( camera_manager_cfg_.func_ != Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW && 
    camera_manager_cfg_.output_format_ == Snapdragon::CameraManagerConfig::OUTPUT_FORMAT_NV12 )
  {
    std::cout << "Snapdragon::CameraManager, INFO, setting output_format nv12" << std::endl;
    camera_params_.set( "preview-format", "nv12" );
  }
  std::cout << "Snapdragon::CameraManager, INFO, set up params " << std::endl;
  int32_t ret; 
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW )
  {
    ret = camera_stereo_->SetParameters( camera_params_ );
  }
  else
#endif
  {
    ret = camera_params_.commit();
  }
  
  
  std::cout << "Snapdragon::CameraManager, INFO, set up params done " << std::endl;
  return ret;
}

int32_t Snapdragon::CameraManager::Initialize()
{
  int32_t rc;
  rc = camera::ICameraDevice::createInstance( cam_id_, &camera_ );
  if( rc != 0 )
  {
    std::cout << "Snapdragon::CameraManager, ERR, could not open camera" << cam_id_ << std::endl;
    return rc;
  }
  camera_->addListener( this );

  rc = camera_params_.init( camera_ );
  if( rc != 0 )
  {
    std::cout << "Snapdragon::CameraManager, ERR, failed to init parameters" << std::endl;
    camera::ICameraDevice::deleteInstance( &camera_ );
  }

  return rc;
}

bool Snapdragon::CameraManager::Init( Snapdragon::CameraManagerConfig& cfg )
{
  camera_manager_cfg_ = cfg;
  int32_t ret;
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  if (cfg.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) {

    std::cout << "Creating stereo device " <<std::endl;
    camera_stereo_ = StereoCameraUtilFactory::createStereoDevice(&camera_params_);
    if (nullptr == camera_stereo_) {
       std::cerr << "Error: Could not create stereo sensor";
      return 1;
    }

    std::cout << "Adding listener" <<std::endl;
    camera_stereo_->addListener(this);
  }
  else
#endif
  {

    int32_t n = camera::getNumberOfCameras();

    std::cout << "Snapdragon::CameraManager, INFO, nucameras = " << n << std::endl;

    if( n < 1 )
    {
      std::cout << "Snapdragon::CameraManager, ERR, No cameras found" << std::endl;
      return false;
    }

    cam_id_ = -1;

    /* find camera based on function */
    for( int32_t i = 0; i < n; i++ )
    {
      camera::CameraInfo info;
      camera::getCameraInfo( i, info );
      std::cout << "Snapdragon::CameraManager, INFO i " << i << " , info.func = " << info.func << std::endl;
      if( info.func == camera_manager_cfg_.func_ )
      {
        cam_id_ = i;
      }
    }

    if( cam_id_ == -1 )
    {
      std::cout << "Snapdragon::CameraManager, ERR, Camera not found" << std::endl;
      return false;
    }


    std::cout << "Snapdragon::CameraManager, INFO, initializing camera id= " << cam_id_ << std::endl;

    ret = Initialize();
    if( ret != 0 )
    {
      std::cout << "Snapdragon::CameraManager, ERR, initializing camera with id " << cam_id_ << " failed with err " << ret << std::endl;
      return false;
    }
  }

  ret = SetParameters();
  if( ret != 0 )
  {
    std::cout << "Snapdragon::CameraManager, ERR, SetParameters camera  failed  with err " << ret << std::endl;
    return false;
  }

  if( camera_manager_cfg_.use_cpa_ )
    cpa_obj_ = mvCPA_Initialize( &camera_manager_cfg_.cpa_config_ );

  
  return true;
}

bool Snapdragon::CameraManager::Deinit()
{
  bool ok = false;
  if( camera_manager_cfg_.use_cpa_ )
    mvCPA_Deinitialize( cpa_obj_ );
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  if (camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) {
    camera_stereo_->removeListener(this); 
    delete camera_stereo_;
  }
  else
#endif
  {
    /* release camera device */
    camera::ICameraDevice::deleteInstance( &camera_ );
  }
  std::cout << "Snapdragon::CameraManager, INFO, Camera deinit done" << std::endl;

  return ok;
}

void Snapdragon::CameraManager::PrintCapabilities()
{
  std::cout << "Snapdragon::CameraManager, INFO, Camera capabilities" << std::endl;
  camera_caps_.p_sizes_ = camera_params_.getSupportedPreviewSizes();
  camera_caps_.v_sizes_ = camera_params_.getSupportedVideoSizes();
  camera_caps_.focus_modes_ = camera_params_.getSupportedFocusModes();
  camera_caps_.wb_modes_ = camera_params_.getSupportedWhiteBalance();
  camera_caps_.iso_modes_ = camera_params_.getSupportedISO();
  camera_caps_.brightness_ = camera_params_.getSupportedBrightness();
  camera_caps_.sharpness_ = camera_params_.getSupportedSharpness();
  camera_caps_.contrast_ = camera_params_.getSupportedContrast();
  camera_caps_.preview_fps_ranges_ = camera_params_.getSupportedPreviewFpsRanges();
  camera_caps_.video_fps_values_ = camera_params_.getSupportedVideoFps();

  std::cout << "Snapdragon::CameraManager, INFO, available preview sizes:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.p_sizes_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, " << i << ": " << camera_caps_.p_sizes_[i].width << " , " << camera_caps_.p_sizes_[i].height << std::endl;
  }
  std::cout << "Snapdragon::CameraManager, INFO, available video sizes:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.v_sizes_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, " << i << ": " << camera_caps_.v_sizes_[i].width << " , " << camera_caps_.v_sizes_[i].height << std::endl;
  }
  std::cout << "Snapdragon::CameraManager, INFO, available focus modes:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.focus_modes_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, " << i << ": " << camera_caps_.focus_modes_[i].c_str() << std::endl;
  }
  std::cout << "Snapdragon::CameraManager, INFO, available whitebalance modes:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.wb_modes_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, " << i << ": " << camera_caps_.wb_modes_[i].c_str() << std::endl;
  }
  std::cout << "Snapdragon::CameraManager, INFO, available ISO modes:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.iso_modes_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, " << i << ": " << camera_caps_.iso_modes_[i].c_str() << std::endl;
  }
  std::cout << "Snapdragon::CameraManager, INFO, available brightness values:" << std::endl;
  std::cout << "Snapdragon::CameraManager, INFO, min=" << camera_caps_.brightness_.min << ", max=" << camera_caps_.brightness_.max << ", step=" << camera_caps_.brightness_.step << std::endl; 

  std::cout << "Snapdragon::CameraManager, INFO, available sharpness values:" << std::endl;
  std::cout << "Snapdragon::CameraManager, INFO, min=" << camera_caps_.sharpness_.min << ", max=" << camera_caps_.sharpness_.max << ", step=" << camera_caps_.sharpness_.step << std::endl;

  std::cout << "Snapdragon::CameraManager, INFO, available contrast values:" << std::endl;
  std::cout << "Snapdragon::CameraManager, INFO, min=" << camera_caps_.contrast_.min << ", max=" << camera_caps_.contrast_.max << ", step=" << camera_caps_.contrast_.step << std::endl;

  std::cout << "Snapdragon::CameraManager, INFO, available preview fps ranges:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.preview_fps_ranges_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, i:" << i <<" min=" << camera_caps_.preview_fps_ranges_[i].min << ", max=" << camera_caps_.preview_fps_ranges_[i].max << std::endl;
  }

  std::cout << "Snapdragon::CameraManager, INFO, available video fps values:" << std::endl;
  for( int32_t i = 0; i < camera_caps_.video_fps_values_.size(); i++ )
  {
    std::cout << "Snapdragon::CameraManager, INFO, i:" << i << " " << camera_caps_.video_fps_values_[i] << std::endl;
  }
}

void Snapdragon::CameraManager::AddNewListener( Snapdragon::CameraFrameListener* listener )
{
  listeners_.push_back( listener );
}


void Snapdragon::CameraManager::onPreviewFrame(camera::ICameraFrame *frame)
{
  CameraFrameType cur_frame;
  float correction = -1000 * GetCameraExposureTime() / 2.0f;
  frame->acquireRef();
  cur_frame.timestamp = frame->timeStamp + (int64_t)correction;
  cur_frame.frame = frame;
  cur_frame.frame_id = frame_id_;

  for( auto listener : listeners_ )
    listener->NewCameraFrameReceived( cur_frame.timestamp, cur_frame.frame->data );

  std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
  if( frame_buffer_.size() == Snapdragon::CameraManagerConfig::MAX_FRAME_BUFFER_SIZE )
  {
    auto nRef = frame_buffer_.begin()->frame->releaseRef();
    frame_buffer_.begin()->frame->releaseRef();
    frame_buffer_.erase( frame_buffer_.begin() );
  }

  // push back the current frame into the buffer
  frame_buffer_.push_back( cur_frame );
  has_new_frame_cv_.notify_all();

  // CPA
  if( camera_manager_cfg_.use_cpa_ && frame_id_ % camera_manager_cfg_.cpa_update_rate_ == 0  )
  {
    ApplyCpa(cur_frame);
  }
  frame_id_++;

}

void Snapdragon::CameraManager::onVideoFrame(camera::ICameraFrame *frame)
{

  CameraFrameType cur_frame;
  float correction = -1000 * GetCameraExposureTime() / 2.0f;
  frame->acquireRef();
  cur_frame.timestamp = frame->timeStamp + (int64_t)correction;
  cur_frame.frame = frame;
  cur_frame.frame_id = frame_id_;

  for( auto listener : listeners_ )
    listener->NewCameraFrameReceived( cur_frame.timestamp, cur_frame.frame->data );

  std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
  if( frame_buffer_.size() == Snapdragon::CameraManagerConfig::MAX_FRAME_BUFFER_SIZE )
  {
    auto nRef = frame_buffer_.begin()->frame->releaseRef();
    frame_buffer_.begin()->frame->releaseRef();

    frame_buffer_.erase( frame_buffer_.begin() );
  }

  // push back the current frame into the buffer
  frame_buffer_.push_back( cur_frame );
  has_new_frame_cv_.notify_all();

  // CPA
  if( camera_manager_cfg_.use_cpa_ && frame_id_ % camera_manager_cfg_.cpa_update_rate_ == 0 ) 
  {
    ApplyCpa(cur_frame);
  }
  frame_id_++;

}

void Snapdragon::CameraManager::WaitForNewFrame()
{
  std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
  has_new_frame_cv_.wait( lck );
}



int64_t Snapdragon::CameraManager::GetNextFrame( int64_t min_frame_id, int64_t* timestamp, unsigned char * frame_buffer, unsigned char * frame_buffer_r )
{
  /// Get latest Frame. 
  int64_t frame_id = -1;
  camera::ICameraFrame * my_frame;
  camera::ICameraFrame * my_frame_r;
  int64_t my_timestamp;
  int64_t my_frame_id;
  {
    std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
    std::list<CameraFrameType>::iterator next_frame = std::find_if( frame_buffer_.begin(), frame_buffer_.end(),
                                     [&]( const CameraFrameType& frame )
    {
      return frame.frame_id > min_frame_id;
    } );

    while( next_frame == frame_buffer_.end() )
    {
      has_new_frame_cv_.wait( lck );

      next_frame = std::find_if( frame_buffer_.begin(), frame_buffer_.end(),
                    [&]( const CameraFrameType& frame )
      {
        return frame.frame_id > min_frame_id;
      } );
    }
    my_frame = next_frame->frame;
    my_frame->acquireRef();
    if( frame_buffer_r != NULL &&
      (camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO ||
        camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) )
    {
      my_frame_r = next_frame->frame_r;
      my_frame_r->acquireRef();
    }
    my_timestamp = next_frame->timestamp;
    my_frame_id = next_frame->frame_id;
  }

  auto n_bytes = Snapdragon::CameraManagerHelper::GetImageSizeInBytes( camera_manager_cfg_.p_size_[0], camera_manager_cfg_.p_size_[1] );
  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO && frame_buffer_r != NULL )
  {
    for( int32_t i = 0; i < camera_manager_cfg_.p_size_[1]; ++i )
    {

      unsigned char* src_l = my_frame->data + i * camera_manager_cfg_.p_size_[0] * 2;
      unsigned char* src_r = src_l + camera_manager_cfg_.p_size_[0];

      memcpy( frame_buffer + i*camera_manager_cfg_.p_size_[0], src_l, camera_manager_cfg_.p_size_[0] );
      memcpy( frame_buffer_r + i*camera_manager_cfg_.p_size_[0], src_r, camera_manager_cfg_.p_size_[0] );
    }
    frame_id = my_frame_id;
  }
  else if( frame_buffer != NULL )
  {
    memcpy( frame_buffer, my_frame->data, n_bytes );
    frame_id = my_frame_id;
  }

  if( frame_id != -1 )
  {
    *timestamp = my_timestamp;
  }

  my_frame->releaseRef(); 
  if( frame_buffer_r != NULL &&
    (camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO ||
      camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) )
  {
    my_frame_r->releaseRef();
  }

  return frame_id;
}


int64_t Snapdragon::CameraManager::GetNextFrame( int64_t min_frame_id, int64_t* timestamp, unsigned char * frame_buffer, unsigned char * frame_buffer_r, Snapdragon::CameraManagerConfig::CameraMangerOutputFormatType output_format)
{
  
  if ( output_format == Snapdragon::CameraManagerConfig::CAM_MAN_OUTPUT_FORMAT_16_BIT_GRAY )
  {
    if ( !(camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW || camera_manager_cfg_.output_format_ == Snapdragon::CameraManagerConfig::OUTPUT_FORMAT_RAW) ) {
      std::cout << "Snapdragon::CameraManager, ERR, Requested format ( output_format =  "<<output_format<<" )is not supported for this configuration " << std::endl;
      return -1;
    }
  }
  /// Get latest Frame. 
  int64_t frame_id = -1;
  camera::ICameraFrame * my_frame;
  camera::ICameraFrame * my_frame_r;
  int64_t my_timestamp;
  int64_t my_frame_id;
  {
    std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
    std::list<CameraFrameType>::iterator next_frame = std::find_if( frame_buffer_.begin(), frame_buffer_.end(),
                                     [&]( const CameraFrameType& frame )
    {
      return frame.frame_id > min_frame_id;
    } );

    while( next_frame == frame_buffer_.end() )
    {
      has_new_frame_cv_.wait( lck );

      next_frame = std::find_if( frame_buffer_.begin(), frame_buffer_.end(),
                    [&]( const CameraFrameType& frame )
      {
        return frame.frame_id > min_frame_id;
      } );
    }
    my_frame = next_frame->frame;
    my_frame->acquireRef();
    if( frame_buffer_r != NULL &&
      camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW )
    {
      my_frame_r = next_frame->frame_r;
      my_frame_r->acquireRef();
    }
    my_timestamp = next_frame->timestamp;
    my_frame_id = next_frame->frame_id;
  }
  
  auto n_bytes = Snapdragon::CameraManagerHelper::GetImageSizeInBytes( camera_manager_cfg_.p_size_[0], camera_manager_cfg_.p_size_[1] );
  if( camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO && frame_buffer_r != NULL)
  {
    for( int32_t i = 0; i < camera_manager_cfg_.p_size_[1]; ++i )
    {
      
      unsigned char* src_l = my_frame->data + i * camera_manager_cfg_.p_size_[0] * 2;
      unsigned char* src_r = src_l + camera_manager_cfg_.p_size_[0];
      memcpy( frame_buffer + i*camera_manager_cfg_.p_size_[0], src_l, camera_manager_cfg_.p_size_[0] );
      memcpy( frame_buffer_r + i*camera_manager_cfg_.p_size_[0], src_r, camera_manager_cfg_.p_size_[0] );
    }
   frame_id = my_frame_id;
  } 
  else if (frame_buffer != NULL) {
    memcpy( frame_buffer, my_frame->data, n_bytes );
    frame_id = my_frame_id;
  }
  else
    std::cout << "Snapdragon::CameraManager, ERR, should not see this message! " << std::endl;

  if( frame_id != -1 )
  {
    *timestamp = my_timestamp;
  }
  my_frame->releaseRef();
  if( frame_buffer_r != NULL &&
      camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW )
  {
    my_frame_r->releaseRef();
  }

  return frame_id;
}

bool Snapdragon::CameraManager::Start()
{
  int32_t ret = 0;
  if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_PREVIEW)
  {
    std::cout << "Snapdragon::CameraManager, INFO, start preview" << std::endl;
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
    if (camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) 
    {
      camera_stereo_->startPreview();     
    }
    else
#endif
    {
      ret = camera_->startPreview();
    }

    if( 0 != ret )
    {
      std::cout << "Snapdragon::CameraManager, ERR, start preview failed" << ret << std::endl;
    }
  }
  else if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_VIDEO)
  {
    std::cout << "Snapdragon::CameraManager, INFO, start recording" << std::endl;
    ret = camera_->startRecording();
    if( 0 != ret )
    {
      std::cout << "Snapdragon::CameraManager, ERR, start recording failed" << ret << std::endl;
    }
  }

  if( 0 != ret )
    return false;

  //Copy values, as SetExposureAndGain only updates if different values
  float tmp_exposure = camera_manager_cfg_.exposure_;
  float tmp_gain = camera_manager_cfg_.gain_;

  camera_manager_cfg_.exposure_ = 0.f;
  camera_manager_cfg_.gain_ = 0.f;

  SetExposureAndGain( tmp_exposure, tmp_gain );

#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  if ((camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW)) 
	{
    camera_params_.setVerticalFlip( true );
    camera_params_.setHorizontalMirror( true );
    camera_stereo_->SetParameters( camera_params_ );
  }
#endif

  
  return true;
}

bool Snapdragon::CameraManager::Stop()
{
  if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_PREVIEW)
  {
    std::cout << "Snapdragon::CameraManager, INFO, stop preview" << std::endl;
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
    if (camera_manager_cfg_.func_ == Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO_RAW) {
      camera_stereo_->stopPreview();

    }
    else
#endif
    {
      camera_->stopPreview();
    }
    std::cout << "Snapdragon::CameraManager, INFO, stop preview done" << std::endl;
  }
  else if( camera_manager_cfg_.capture_mode_ == Snapdragon::CameraManagerConfig::CAPTURE_MODE_VIDEO)
  {
    std::cout << "Snapdragon::CameraManager, INFO, stop recording" << std::endl;
    camera_->stopRecording();
    std::cout << "Snapdragon::CameraManager, INFO, stop recording done" << std::endl;
  }
  return true;
}

#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
void Snapdragon::CameraManager::onPreviewFrame(std::vector<camera::ICameraFrame *> frames) {
  
  char name[50];
  int32_t time_diff = 0;
  char format_str[10]="raw";


  /*ToDo: enable manual exposure, gain control */
  float correction = -1000 * GetCameraExposureTime() / 2.0f;
  CameraFrameType cur_frame;
  cur_frame.timestamp = frames[0]->timeStamp + (int64_t)correction;
  cur_frame.frame = frames[0];
  cur_frame.frame_r = frames[1];
  cur_frame.frame->acquireRef();
  cur_frame.frame_r->acquireRef();
  cur_frame.frame_id = frame_id_;


  std::unique_lock<std::mutex> lck( frame_buffer_mtx_ );
  if( frame_buffer_.size() == Snapdragon::CameraManagerConfig::MAX_FRAME_BUFFER_SIZE )
  {
    auto nRef = frame_buffer_.begin()->frame->releaseRef();
    frame_buffer_.begin()->frame_r->releaseRef();
    frame_buffer_.erase( frame_buffer_.begin() );
  }

  // push back the current frame into the buffer
  frame_buffer_.push_back( cur_frame );
  has_new_frame_cv_.notify_all();


#ifdef ENABLE_CPA_FOR_STEREO_RAW
  // CPA
  if( camera_manager_cfg_.use_cpa_ && frame_id_ % camera_manager_cfg_.cpa_update_rate_ == 0  && camera_manager_cfg_.func_ != Snapdragon::CameraManagerConfig::CAM_FUNC_STEREO )
  {
    ApplyCpa(cur_frame);
  }

#endif 
  frame_id_++;
  
}
void Snapdragon::CameraManager::onVideoFrame(std::vector<camera::ICameraFrame *> frames) 
{
}
#endif
Snapdragon::CameraFrameListener::~CameraFrameListener()
{};
