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
        minDisparities = 0;
        numDisparities = 256;
        postProcessDisparity = true;
        postProcessPointCloud = true;
        medianWindowSize = 3;
        invalidFlag = true;
        invalidC = 0;
        maxSpeckleSize = 40;
        maxDiff = 4;
        filteringWindowSize = 3;
        filteringDistance = 0.1;
        uniquenessRatio = 10;
        rawLeftTransmitEnabled = false;
        rawRightTransmitEnabled = false;
        rectLeftTransmitEnabled = true;
        rectRightTransmitEnabled = false;
        disparityTransmitEnabled = true;
        enablePointCloudOutput = true;
        SGBMP1 = 5;
        SGBMP2 = 60;
        pixelFormat = PixelFormat_RGB8Packed;
        disparityScaleFactor = 1.0;
    }

    std::string StereoParameters::ToString()
    {
        std::stringstream strstr("");

        strstr << "exposureTime " << exposureTime << std::endl;
        strstr << "gainValue " << gainValue << std::endl;
        strstr << "focalLength " << focalLength << std::endl;
        strstr << "baseline " << baseline << std::endl;
        strstr << "fCurrentCentreRow " << fCurrentCentreRow << std::endl;
        strstr << "fCurrentCentreCol " << fCurrentCentreCol << std::endl;
        strstr << "minDisparities " << minDisparities << std::endl;
        strstr << "numDisparities " << numDisparities << std::endl;
        strstr << "postProcessDisparity " << postProcessDisparity << std::endl;
        strstr << "postProcessPointCloud " << postProcessPointCloud << std::endl;
        strstr << "medianWindowSize " << medianWindowSize << std::endl;
        strstr << "speckleRange " << speckleRange << std::endl;
        strstr << "filteringWindowSize " << filteringWindowSize << std::endl;
        strstr << "filteringDistance " << filteringDistance << std::endl;
        strstr << "uniquenessRatio " << uniquenessRatio << std::endl;
        strstr << "rawLeftTransmitEnabled " << rawLeftTransmitEnabled << std::endl;
        strstr << "rawRightTransmitEnabled " << rawRightTransmitEnabled << std::endl;
        strstr << "rectLeftTransmitEnabled " << rectLeftTransmitEnabled << std::endl;
        strstr << "rectRightTransmitEnabled " << rectRightTransmitEnabled << std::endl;
        strstr << "disparityTransmitEnabled " << disparityTransmitEnabled << std::endl;
        strstr << "enablePointCloudOutput " << enablePointCloudOutput << std::endl;
        strstr << "SGBMP1 " << SGBMP1 << std::endl;
        strstr << "SGBMP2 " << SGBMP2 << std::endl;
        strstr << "pixelFormat " << pixelFormat << std::endl;
        strstr << "disparityScaleFactor " << disparityScaleFactor << std::endl;

        return strstr.str();
    }
} // namespace SpinStereo
