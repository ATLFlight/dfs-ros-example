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

#include <thread>
#include <vector>
#include <mvDFS.h>
#include "SnapdragonCameraManager.hpp"

namespace Snapdragon {
  class DfsManager;
}

class Snapdragon::DfsManager
{
public:
  struct DfsCamConfiguration
  {
    mvStereoConfiguration stereo_config;  // Configuration for stereo cameras
    MV_MODE dfs_mode;             // whether to run on GPU or not.
    std::vector<uint16_t>dfs_masks;     // DFS mask boundaries , number of elements(4* numDFSMaskingRegions).
    int16_t max_disparity;       // Maximum disparity in pixels
    int16_t min_disparity;       // Minimum disparity in pixels
    Snapdragon::CameraManagerConfig::CameraMangerOutputFormatType camera_manager_image_format;
    DfsCamConfiguration() : dfs_mode( MV_MODE_GPU ), max_disparity(32), min_disparity(0)
    {
    }
  };

  DfsManager();
  virtual ~DfsManager();

  /* intializes the DfsManager with appropriate config */
  bool Init( Snapdragon::CameraManager* camera_manager, DfsCamConfiguration* dfs_cam_config );

  /* de intializes the DfsManager */
  bool Deinit();

  /* starts DfsManager processing */
  bool Start();

  /* stops DfsManager processing */
  bool Stop();
  
  /* does the main function calls to get the current images and get the disparity and depth images */
  void DfsCamProcessingMain();

  /* get the current frame buffers */
  unsigned char* GetCurFrameBufferL() {return cur_frame_buffer_l_;}
  unsigned char* GetCurFrameBufferR() {return cur_frame_buffer_r_;}
  uint16_t* GetCurDisparity() {return disparity_;}
  float32_t* GetCurInvDepth() {return inv_depth_;}
  DfsCamConfiguration GetDfsCamConfig() {return dfs_cam_config_;}
  mvCameraConfiguration GetDepthCamera() {return depth_camera_;}
   
  mvDFS* mv_dfs_ptr_;
  Snapdragon::CameraManager* camera_manager_ptr_;
  bool initialized_;

private:

  int64_t cur_frame_id_, min_frame_id_;
  int64_t cur_timestamp_;
  size_t n_bytes_;
  unsigned char * cur_frame_buffer_l_;
  unsigned char * cur_frame_buffer_r_;
  uint16_t* disparity_;
  float32_t* inv_depth_;
  DfsCamConfiguration dfs_cam_config_;
  mvCameraConfiguration depth_camera_;
   
};
