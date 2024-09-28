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

#include "stereo_image_publisher/StereoParameters.h"
#include <sstream>

namespace SpinStereo
{
    StereoParameters::StereoParameters()
    {
        exposureTime = 20000;
        gainValue = 0.0;
        focalLength = 0.0;
        baseline = 0.0;
        fCurrentCentreRow = 0.0;
        fCurrentCentreCol = 0.0;
        minDisparity = 0.0;
        numDisparities = 256;
        doInvalidDisparityCheck = true;
        invalidDisparityValue = 0.0;
        postProcessDisparity = true;
        maxSpeckleSize = 40;
        maxDiff = 4.0f;
        uniquenessRatio = 10;
        streamTransmitFlags = {false, false, true, false, true};
        doComputePointCloud = true;
        SGBMP1 = 5;
        SGBMP2 = 60;
        pixelFormat = PixelFormat_RGB8Packed;
        disparityScaleFactor = 1.0;
    }

    std::string StereoParameters::ToString() const
    {
        std::stringstream strstr("");

        strstr << "exposureTime " << exposureTime << std::endl;
        strstr << "gainValue " << gainValue << std::endl;
        strstr << "focalLength " << focalLength << std::endl;
        strstr << "baseline " << baseline << std::endl;
        strstr << "fCurrentCentreRow " << fCurrentCentreRow << std::endl;
        strstr << "fCurrentCentreCol " << fCurrentCentreCol << std::endl;
        strstr << "minDisparity " << minDisparity << std::endl;
        strstr << "numDisparities " << numDisparities << std::endl;
        strstr << "doInvalidDisparityCheck " << doInvalidDisparityCheck << std::endl;
        strstr << "invalidDisparityValue " << invalidDisparityValue << std::endl;
        strstr << "postProcessDisparity " << postProcessDisparity << std::endl;
        strstr << "maxSpeckleSize " << maxSpeckleSize << std::endl;
        strstr << "maxDiff " << maxDiff << std::endl;
        strstr << "uniquenessRatio " << uniquenessRatio << std::endl;
        strstr << streamTransmitFlags.ToString() << std::endl;
        strstr << "doComputePointCloud " << doComputePointCloud << std::endl;
        strstr << "SGBMP1 " << SGBMP1 << std::endl;
        strstr << "SGBMP2 " << SGBMP2 << std::endl;
        strstr << "pixelFormat " << pixelFormat << std::endl;
        strstr << "disparityScaleFactor " << disparityScaleFactor << std::endl;

        return strstr.str();
    }
} // namespace SpinStereo