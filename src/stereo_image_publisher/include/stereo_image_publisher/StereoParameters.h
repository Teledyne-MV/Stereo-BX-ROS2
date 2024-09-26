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

#pragma once

#include "Spinnaker.h"
#include <string>
#include <fstream>
#include <iostream>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;

namespace SpinStereo
{
    /** @addtogroup stereo_control
     * @{
     */

    /**
     * @class StereoParameters
     * @brief Class for handling parameters of the S3D camera.
     */
    class StereoParameters
    {
      public:
        /**
         * @brief Constructor for StereoParameters.
         */
        StereoParameters();

        /**
         * @brief Converts the parameters to a string representation.
         * @return A string representation of the parameters.
         */
        std::string ToString();

        unsigned int minDisparities; ///< Minimum number of disparities.
        unsigned int numDisparities; ///< Number of disparities.

        bool rawLeftTransmitEnabled;   ///< Flag to enable raw left image transmission.
        bool rawRightTransmitEnabled;  ///< Flag to enable raw right image transmission.
        bool rectLeftTransmitEnabled;  ///< Flag to enable rectified left image transmission.
        bool rectRightTransmitEnabled; ///< Flag to enable rectified right image transmission.
        bool disparityTransmitEnabled; ///< Flag to enable disparity image transmission.
        bool enablePointCloudOutput;

        int speckleThreshold; ///< Speckle threshold value.
        int uniquenessRatio;  ///< Uniqueness ratio value.
        int SGBMP1;           ///< SGBM P1 parameter.
        int SGBMP2;           ///< SGBM P2 parameter.

        bool postProcessDisparity;  ///< Flag to enable disparity post-processing.
        bool postProcessPointCloud; ///< Flag to enable point cloud post-processing.

        int medianWindowSize;    ///< Median filter window size.
        int speckleRange;        ///< Speckle range value.
        int filteringWindowSize; ///< Filtering window size.
        float filteringDistance; ///< Filtering distance value.
        int maxSpeckleSize;
        double maxDiff;

        float exposureTime;           ///< Exposure time value.
        float gainValue;              ///< Gain value.
        float focalLength;
        float baseline;
        float fCurrentCentreRow;
        float fCurrentCentreCol;
        PixelFormatEnums pixelFormat; ///< Pixel format enumeration.
        bool invalidFlag;
        double invalidC;


        float disparityScaleFactor;
    };

    /** @} */ // end of stereo_control
} // namespace SpinStereo

#endif // STEREO_PARAMETERS_H
