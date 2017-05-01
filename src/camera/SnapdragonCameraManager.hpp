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

#include <camera.h>
#include <camera_parameters.h>
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW  // To enable stereo through RDI, define to 1 in CMakeLists.txt. Undefined by default. Currently not supported for Snapdragon Flight.
#include <camera_util_stereo.h>
#endif
#include "SnapdragonCameraManagerConfig.hpp"
#include "SnapdragonCameraManagerHelper.hpp"
#include "mv/mvCPA.h"
#include "mv/mvSRW.h"
#include <list>
#include <mutex>
#include <condition_variable>

namespace Snapdragon {
  class CameraManager;
  class CameraFrameListener;
}

class Snapdragon::CameraFrameListener
{
public:
  virtual void NewCameraFrameReceived( int64_t, unsigned char * ) = 0;
  virtual ~CameraFrameListener() = 0;
};

class Snapdragon::CameraManager : public camera::ICameraListener 
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  ,
  public ICameraDeviceArrayListener
#endif
{
public:

  struct CameraFrameType
  {
    camera::ICameraFrame *frame;    // pointer to left camera frame
    camera::ICameraFrame *frame_r;  // pointer to right camera frame if using stereo
    int64_t timestamp;      // timestamp of camera frame in ns 
    int64_t frame_id;       // count since first camera frame
  };

  CameraManager();
  ~CameraManager();

  /* intializes the camera manager with appropriate config */
  bool Init( Snapdragon::CameraManagerConfig& cfg );

  /* de intializes the camera manager */
  bool Deinit();

  /* starts the camera */
  bool Start();

  /* stops the camera */
  bool Stop();

  /* blocking function call that waits for next available frame
    min_frame_id: wait for frame that has frame ID higher than this
    timestamp: stores the timestamp of the returned frame
    frame_buffer: buffer allocated by the caller to store the returned frame
    frame_buffer_r: buffer allocated by the caller to store the returned right frame. Set to NULL if not a stereo camera.
    output_format: allows you to choose between 10-bit RAW (stored as 16 bits per pixel) and 8-bit YUV. 
  */
  int64_t GetNextFrame( int64_t min_frame_id, int64_t* timestamp, unsigned char * frame_buffer, unsigned char * frame_buffer_r = NULL );
  int64_t GetNextFrame( int64_t min_frame_id, int64_t* timestamp, unsigned char * frame_buffer, unsigned char * frame_buffer_r, Snapdragon::CameraManagerConfig::CameraMangerOutputFormatType output_format);

  /* listener methods from camera::ICameraListener */
  virtual void onError();
  virtual void onPreviewFrame(camera::ICameraFrame* frame);
  virtual void onVideoFrame(camera::ICameraFrame* frame);

#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  virtual void onPreviewFrame(std::vector<camera::ICameraFrame *> frames);
  virtual void onVideoFrame(std::vector<camera::ICameraFrame *> frames);
#endif	
  void PrintCapabilities();
  void EnableCpaLogging( mvSRW_Writer * writer );
  void AddNewListener( Snapdragon::CameraFrameListener* listener );
  const Snapdragon::CameraManagerConfig& GetCameraConfig() const { return camera_manager_cfg_ ; }

private:
  int32_t Initialize();
  int32_t SetParameters();
  void SetExposureAndGain( float exposure, float gain );
  float GetCameraExposureTime();
  void WaitForNewFrame();
  void ApplyCpa(CameraFrameType cur_frame);

  Snapdragon::CameraManagerConfig camera_manager_cfg_;
  camera::CameraParams camera_params_;
  CameraCapabilities camera_caps_;
  camera::ICameraDevice* camera_;
#ifdef ENABLE_FUNC_CAMERA_STEREO_RAW
  camera::ICameraDeviceArray *camera_stereo_;
#endif
  int32_t cam_id_;
  size_t frame_id_;
  std::vector<Snapdragon::CameraFrameListener*> listeners_;
  float camera_exposure_hist_[2];

  std::mutex frame_buffer_mtx_;
  std::condition_variable has_new_frame_cv_;
  std::list<CameraFrameType> frame_buffer_;

  mvCPA* cpa_obj_;
  mvSRW_Writer * srw_obj_;
};
