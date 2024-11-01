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

#ifndef STEREO_PARAMETERS_H
#define STEREO_PARAMETERS_H

#include <fstream>
#include <iostream>
#include <string>
#include "Spinnaker.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace SpinStereo {
struct StreamTransmitFlags {
  bool raw_sensor_1_transmit_enabled;   ///< Flag to enable raw sensor1 image
                                        ///< transmission.
  bool raw_sensor_2_transmit_enabled;   ///< Flag to enable raw sensor2 image
                                        ///< transmission.
  bool rect_sensor_1_transmit_enabled;  ///< Flag to enable rectified sensor1
                                        ///< image transmission.
  bool rect_sensor_2_transmit_enabled;  ///< Flag to enable rectified sensor2
                                        ///< image transmission.
  bool disparity_transmit_enabled;  ///< Flag to enable disparity image
                                    ///< transmission.

  std::string ToString() const {
    std::stringstream strstr("");

    strstr << "raw_sensor_1_transmit_enabled " << raw_sensor_1_transmit_enabled
           << std::endl;
    strstr << "raw_sensor_2_transmit_enabled " << raw_sensor_2_transmit_enabled
           << std::endl;
    strstr << "rect_sensor_1_transmit_enabled "
           << rect_sensor_1_transmit_enabled << std::endl;
    strstr << "rect_sensor_2_transmit_enabled "
           << rect_sensor_2_transmit_enabled << std::endl;
    strstr << "disparity_transmit_enabled " << disparity_transmit_enabled;

    return strstr.str();
  };
};

/**
 * @class StereoParameters
 * @brief Class for handling parameters of the S3D camera.
 */
class StereoParameters {
 public:
  /**
   * @brief Constructor for StereoParameters.
   */
  StereoParameters();

  /**
   * @brief Converts the parameters to a string representation.
   * @return A string representation of the parameters.
   */
  std::string ToString() const;

  float scan3d_coordinate_offset;  ///< Minimum number of disparities.

  StreamTransmitFlags
      stream_transmit_flags;  ///< Flags to enable streams image transmission.

  bool do_compute_point_cloud;  ///< flag to enable computation of the 3D point
                                ///< cloud.

  int uniqueness_ratio;  ///< Uniqueness ratio value.
  int small_penalty;     ///< Small penalty.
  int large_penalty;     ///< Large penalty.

  bool post_process_disparity;  ///< Flag to enable disparity post-processing.

  int max_speckle_size;  ///< Speckle range value.
  double max_diff;       ///< Speckle threshold value.

  bool auto_exposure;
  float exposure_time;  ///< Exposure time value.
  bool auto_gain;
  float gain_value;  ///< Gain value.
  bool acquisition_frame_rate_enabled;
  float frame_rate;
};
}  // namespace SpinStereo

#endif  // STEREO_PARAMETERS_H