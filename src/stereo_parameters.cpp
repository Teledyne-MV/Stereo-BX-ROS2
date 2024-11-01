//=============================================================================
// Copyright (c) 2024 FLIR Integrated Imaging Solutions, Inc. All Rights
// Reserved
//
// This software is the confidential and proprietary information of FLIR
// Integrated Imaging Solutions, Inc. ("Confidential Information"). You
// shall not disclose such Confidential Information and shall use it only in
// accordance with the terms of the non-disclosure or license agreement you
// entered into with FLIR Integrated Imaging Solutions, Inc. (FLIR).
//
// FLIR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. FLIR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#include "stereo_image_publisher/stereo_parameters.hpp"
#include <sstream>

namespace SpinStereo {
StereoParameters::StereoParameters() {
  // camera
  exposure_time = 20000;
  gain_value = 0.0;
  acquisition_frame_rate_enabled = true;
  frame_rate = 5.0;
  // post processing
  post_process_disparity = true;
  max_speckle_size = 40;
  max_diff = 4.0;
  // stream settings
  stream_transmit_flags = {false, false, true, false, true};
  // stereo settings
  scan3d_coordinate_offset = 0.0;
  small_penalty = 5;
  large_penalty = 60;
  uniqueness_ratio = 10;
  // point cloud settings
  do_compute_point_cloud = true;
}

std::string StereoParameters::ToString() const {
  std::stringstream strstr("");

  strstr << "exposure_time " << exposure_time << std::endl;
  strstr << "gain_value " << gain_value << std::endl;
  strstr << "frame_rate " << frame_rate << std::endl;
  strstr << "scan3d_coordinate_offset " << scan3d_coordinate_offset
         << std::endl;
  strstr << "post_process_disparity " << post_process_disparity << std::endl;
  strstr << "max_speckle_size " << max_speckle_size << std::endl;
  strstr << "max_diff " << max_diff << std::endl;
  strstr << "uniqueness_ratio " << uniqueness_ratio << std::endl;
  strstr << stream_transmit_flags.ToString() << std::endl;
  strstr << "do_compute_point_cloud " << do_compute_point_cloud << std::endl;
  strstr << "small penalty " << small_penalty << std::endl;
  strstr << "large penalty " << large_penalty << std::endl;

  return strstr.str();
}
}  // namespace SpinStereo