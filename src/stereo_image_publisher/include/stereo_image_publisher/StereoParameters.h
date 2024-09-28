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

    struct StreamTransmitFlags
    {
        bool rawLeftTransmitEnabled;   ///< Flag to enable raw left image transmission.
        bool rawRightTransmitEnabled;  ///< Flag to enable raw right image transmission.
        bool rectLeftTransmitEnabled;  ///< Flag to enable rectified left image transmission.
        bool rectRightTransmitEnabled; ///< Flag to enable rectified right image transmission.
        bool disparityTransmitEnabled; ///< Flag to enable disparity image transmission.

        std::string ToString() const
        {
            std::stringstream strstr("");

            strstr << "rawLeftTransmitEnabled " << rawLeftTransmitEnabled << std::endl;
            strstr << "rawRightTransmitEnabled " << rawRightTransmitEnabled << std::endl;
            strstr << "rectLeftTransmitEnabled " << rectLeftTransmitEnabled << std::endl;
            strstr << "rectRightTransmitEnabled " << rectRightTransmitEnabled << std::endl;
            strstr << "disparityTransmitEnabled " << disparityTransmitEnabled;

            return strstr.str();
        };
    };

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
        std::string ToString() const;

        float minDisparity; ///< Minimum number of disparities.
        unsigned int numDisparities; ///< Number of disparities.
        bool doInvalidDisparityCheck;
        float invalidDisparityValue;

        StreamTransmitFlags streamTransmitFlags; ///< Flags to enable streams image transmission.

        bool doComputePointCloud; ///< flag to enable computation of the 3D point cloud.

        int uniquenessRatio;  ///< Uniqueness ratio value.
        int SGBMP1;           ///< SGBM P1 parameter.
        int SGBMP2;           ///< SGBM P2 parameter.

        bool postProcessDisparity;  ///< Flag to enable disparity post-processing.

        int medianWindowSize;    ///< Median filter window size.
        int maxSpeckleSize;        ///< Speckle range value.
        double maxDiff;

        float exposureTime; ///< Exposure time value.
        float gainValue;    ///< Gain value.
        float focalLength;
        float baseline;
        float fCurrentCentreRow;
        float fCurrentCentreCol;
        PixelFormatEnums pixelFormat; ///< Pixel format enumeration.

        float disparityScaleFactor;
    };

    /** @} */ // end of stereo_control
} // namespace SpinStereo

#endif // STEREO_PARAMETERS_H