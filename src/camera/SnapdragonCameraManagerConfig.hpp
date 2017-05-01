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
#include <mv/mvCPA.h>

namespace Snapdragon {
  class CameraCapabilities;
  class CameraManagerConfig;
}

class Snapdragon::CameraCapabilities
{
public:
  std::vector<camera::ImageSize> p_sizes_, v_sizes_;
  std::vector<std::string> focus_modes_, wb_modes_, iso_modes_;
  camera::Range brightness_, sharpness_, contrast_;
  std::vector<camera::Range> preview_fps_ranges_;
  std::vector<camera::VideoFPS> video_fps_values_;
};

class Snapdragon::CameraManagerConfig
{

public:

  enum OutputFormatType
  {
    OUTPUT_FORMAT_YUV,
    OUTPUT_FORMAT_RAW,  // currently not supported for stereo
    OUTPUT_FORMAT_NV12,
  };

  enum CameraMangerOutputFormatType
  {
    CAM_MAN_OUTPUT_FORMAT_8_BIT_GRAY,
    CAM_MAN_OUTPUT_FORMAT_16_BIT_GRAY,
  };

  enum CamFunction
  {
    CAM_FUNC_HIRES = 0,
    CAM_FUNC_OPTIC_FLOW = 1,
    CAM_FUNC_RIGHT_SENSOR = 2,
    CAM_FUNC_STEREO = 3,
    CAM_FUNC_LEFT_SENSOR = 4,
    CAM_FUNC_STEREO_RAW = 5,  // this mode is currently not supported
  };

  enum CaptureMode
  {
    CAPTURE_MODE_PREVIEW,
    CAPTURE_MODE_VIDEO
  };

  enum
  {
    MIN_EXPOSURE_VALUE = 0,
    // This value comes (FrameLength - 20)
    // FrameLength is set at 1716 for 30fps camera
    // Row period = 19.333 us
    // Exposure time = RowPeriod * ExposureValue
    MAX_EXPOSURE_VALUE = 1696,
    MIN_GAIN_VALUE = 0,
    MAX_GAIN_VALUE = 255,
    MAX_FRAME_BUFFER_SIZE = 4
  };

  enum CaptureResolution
  {
    CAPTURE_RESOLUTION_QVGA_320_240 = 0,
    CAPTURE_RESOLUTION_VGA_640_480 = 1,
    CAPTURE_RESOLUTION_HD_1280_720 = 2,
    CAPTURE_RESOLUTION_FHD_1920_1080 = 3,
    CAPTURE_RESOLUTION_UHD_3840_2160 = 4
  };

  CameraManagerConfig()
  {
    // default values
    gain_ = 1.f;
    exposure_ = 1.0f;
    afmode_ = 4;
    cameraid_ = 0;
    frame_rate_ = 30.0f;
    use_cpa_ = false;
    cpa_update_rate_ = 4; /// In tests we notice that there is a delay of 3 frames for the camera parameters to get applied.Hence
               /// we wait for 3 frames before computing and applying new cpa values.
    capture_mode_ = CAPTURE_MODE_PREVIEW;
    output_format_ = OUTPUT_FORMAT_YUV;
    row_period_us_ = 19.3333f;
    resolution_ = CAPTURE_RESOLUTION_VGA_640_480;
    p_size_.resize( 2 );
    p_size_[0] = 640;
    p_size_[1] = 480;
    p_stride_ = 640;
  }

  float gain_;
  float exposure_;
  int32_t afmode_;
  int32_t cameraid_;
  float frame_rate_;
  std::vector<int> p_size_;
  int32_t p_stride_;
  CaptureMode capture_mode_;
  CamFunction func_;
  OutputFormatType output_format_;
  CaptureResolution resolution_;

  bool use_cpa_;
  int32_t cpa_update_rate_;
  mvCPA_Configuration cpa_config_;
  float row_period_us_;
};
