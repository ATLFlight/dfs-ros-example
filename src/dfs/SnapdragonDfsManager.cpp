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

#include "SnapdragonDfsManager.hpp"
#include <math.h>
#include <string.h>

Snapdragon::DfsManager::DfsManager()
{
  mv_dfs_ptr_ = NULL;
  disparity_ = NULL;
  inv_depth_ = NULL;
  initialized_ = false;
}

Snapdragon::DfsManager::~DfsManager()
{
}

bool Snapdragon::DfsManager::Init( DfsCamConfiguration* dfs_cam_config, int32_t height, int32_t width )
{

  if( dfs_cam_config == NULL )
  {  
    std::cout << "Snapdragon::DfsManager, ERR, dfs_cam_config is NULL " << std::endl;
    return false;
  }
  
  if( !initialized_ )
  {
    auto num_pixels = height * width;
    
    // compute disparity and depth
    disparity_ = new uint16_t[num_pixels];
    if( disparity_ == NULL )
    {
      std::cout << "Snapdragon::DfsManager, ERR, couldnt allocate pixels for disparity " << num_pixels << std::endl;
      return false;
    }
    memset( disparity_, 0, num_pixels * sizeof( uint16_t ) );

    inv_depth_ = new float[num_pixels];
    if( inv_depth_ == NULL )
    {
      std::cout << "Snapdragon::DfsManager, ERR, couldnt allocate pixels for invDepth " << num_pixels << std::endl;
      return false;
    }
    memset( inv_depth_, 0, num_pixels * sizeof( float32_t ) );
    dfs_cam_config_ = *dfs_cam_config;
    bool use_10_bit_grayscale = false;
    mv_dfs_ptr_ = mvDFS_Initialize( &dfs_cam_config_.stereo_config, dfs_cam_config_.dfs_mode, use_10_bit_grayscale );
    
    if( mv_dfs_ptr_ == NULL )
    {
      std::cout << "Snapdragon::DfsManager, ERR, failed initializing mvDFS " << std::endl;
      return false;
    }

    // how to get depth camera parameters if needed
    memset( &depth_camera_, 0, sizeof( depth_camera_ ) );
    mvDFS_GetDepthCameraConfiguration( mv_dfs_ptr_, &depth_camera_ );

    initialized_ = true;
  }

  return initialized_;
}

bool Snapdragon::DfsManager::Deinit()
{
  mvDFS_Deinitialize( mv_dfs_ptr_ );
  if( disparity_ )
    delete[] disparity_;
  if( inv_depth_ )
    delete[] inv_depth_;
  return true;
}

void Snapdragon::DfsManager::Process(const uint8_t* cur_frame_buffer_l, const uint8_t* cur_frame_buffer_r)
{
  if (!initialized_) {
    std::cout << "Snapdragon::DfsManager, ERR, tried to call DfsCamProcessingMain, but DfsManager is not initialized " << std::endl;
    return;
  }
  mvDFS_GetDepths( mv_dfs_ptr_, cur_frame_buffer_l, cur_frame_buffer_r, 0, nullptr, dfs_cam_config_.min_disparity, dfs_cam_config_.max_disparity, disparity_, inv_depth_ );
}
