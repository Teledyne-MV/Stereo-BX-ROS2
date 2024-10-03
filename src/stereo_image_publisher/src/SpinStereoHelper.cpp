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
#include "stereo_image_publisher/SpinStereoHelper.h"
#include <iostream>
#include <sstream>
#include <thread>
#include <cmath>

#include <algorithm>
#include <vector>

using namespace std;

const unsigned int printEveryNFrames = 50;
unsigned int imageGroupCounter = 0;

namespace SpinStereo
{
    bool IsDeviceBumblebeeX(CameraPtr pCam)
    {
        if (IsReadable(pCam->TLDevice.DeviceModelName))
        {
            std::string deviceModelName = pCam->TLDevice.DeviceModelName.ToString().c_str();
            std::string deviceModelPrefix("Bumblebee X");

            std::size_t found = deviceModelName.find(deviceModelPrefix);
            if (found != std::string::npos)
            {
                // The device is a Bumblebee camera
                return true;
            }
        }
        else
        {
            cerr << "DeviceModelName is not readable" << endl;
        }
        return false;
    }

    gcstring GetSerialNumber(CameraPtr pCam)
    {
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        gcstring deviceSerialNumber("");
        CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
        if (IsReadable(ptrStringSerial))
        {
            deviceSerialNumber = ptrStringSerial->GetValue();
        }

        return deviceSerialNumber;
    }

    // Disables or enables heartbeat on GEV cameras so debugging does not incur timeout errors
    bool ConfigureGVCPHeartbeat(CameraPtr pCam, bool enableHeartbeat)
    {
        //
        // Write to boolean node controlling the camera's heartbeat
        //
        // *** NOTES ***
        // This applies only to GEV cameras.
        //
        // GEV cameras have a heartbeat built in, but when debugging applications the
        // camera may time out due to its heartbeat. Disabling the heartbeat prevents
        // this timeout from occurring, enabling us to continue with any necessary
        // debugging.
        //
        // *** LATER ***
        // Make sure that the heartbeat is reset upon completion of the debugging.
        // If the application is terminated unexpectedly, the camera may not locked
        // to Spinnaker indefinitely due to the the timeout being disabled.  When that
        // happens, a camera power cycle will reset the heartbeat to its default setting.

        // Retrieve TL device nodemap
        INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        CEnumerationPtr ptrDeviceType = nodeMapTLDevice.GetNode("DeviceType");
        if (!IsReadable(ptrDeviceType))
        {
            return false;
        }

        if (ptrDeviceType->GetIntValue() != DeviceType_GigEVision)
        {
            return true;
        }

        if (enableHeartbeat)
        {
            cout << endl << "Resetting heartbeat..." << endl << endl;
        }
        else
        {
            cout << endl << "Disabling heartbeat..." << endl << endl;
        }

        CBooleanPtr ptrDeviceHeartbeat = nodeMap.GetNode("GevGVCPHeartbeatDisable");
        if (!IsWritable(ptrDeviceHeartbeat))
        {
            cout << "Unable to configure heartbeat. Continuing with execution as this may be non-fatal..." << endl
                 << endl;
        }
        else
        {
            ptrDeviceHeartbeat->SetValue(!enableHeartbeat);

            if (!enableHeartbeat)
            {
                cout << "WARNING: Heartbeat has been disabled for the rest of this example run." << endl;
                cout << "         Heartbeat will be reset upon the completion of this run.  If the " << endl;
                cout << "         example is aborted unexpectedly before the heartbeat is reset, the" << endl;
                cout << "         camera may need to be power cycled to reset the heartbeat." << endl << endl;
            }
            else
            {
                cout << "Heartbeat has been reset." << endl;
            }
        }

        return true;
    }

    bool ResetGVCPHeartbeat(CameraPtr pCam)
    {
        return ConfigureGVCPHeartbeat(pCam, true);
    }

    bool DisableGVCPHeartbeat(CameraPtr pCam)
    {
        return ConfigureGVCPHeartbeat(pCam, false);
    }

    bool SetAcquisitionMode(INodeMap& nodeMap)
    {
        //
        // Set acquisition mode to continuous
        //
        // *** NOTES ***
        // Because the example acquires and saves 10 images, setting acquisition
        // mode to continuous lets the example finish. If set to single frame
        // or multiframe (at a lower number of images), the example would just
        // hang. This would happen because the example has been written to
        // acquire 10 images while the camera would have been programmed to
        // retrieve less than that.
        //
        // Setting the value of an enumeration node is slightly more complicated
        // than other node types. Two nodes must be retrieved: first, the
        // enumeration node is retrieved from the nodemap; and second, the entry
        // node is retrieved from the enumeration node. The integer value of the
        // entry node is then set as the new value of the enumeration node.
        //
        // Notice that both the enumeration and the entry nodes are checked for
        // availability and readability/writability. Enumeration nodes are
        // generally readable and writable whereas their entry nodes are only
        // ever readable.
        //
        // Retrieve enumeration node from nodemap
        CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
        if (!IsReadable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
        {
            cout << "Unable to set acquisition mode to continuous (enum retrieval). Aborting..." << endl << endl;
            return false;
        }

        // Retrieve entry node from enumeration node
        CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
        if (!IsReadable(ptrAcquisitionModeContinuous))
        {
            cout << "Unable to get or set acquisition mode to continuous (entry retrieval). Aborting..." << endl
                 << endl;
            return false;
        }

        // Retrieve integer value from entry node
        const int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

        // Set integer value from entry node as new value of enumeration node
        ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

        return true;
    }

    bool SetStreamBufferHandlingMode(INodeMap& nodeMapTLDevice)
    {
        // Set the StreamBufferHandlingMode
        CEnumerationPtr ptrHandlingMode = nodeMapTLDevice.GetNode("StreamBufferHandlingMode");
        if (!IsReadable(ptrHandlingMode) || !IsWritable(ptrHandlingMode))
        {
            cout << "Unable to read or write to StreamBufferHandlingMode node. Aborting..." << endl << endl;
            return false;
        }

        cout << "Set Handling mode to OldestFirst" << endl;
        CEnumEntryPtr ptrHandlingModeEntry = ptrHandlingMode->GetEntryByName("OldestFirst");
        if (!IsReadable(ptrHandlingModeEntry))
        {
            cout << "Unable to read ptrHandlingModeEntry node. Aborting..." << endl << endl;
            return false;
        }

        int64_t ptrHandlingModeEntryVal = ptrHandlingModeEntry->GetValue();
        ptrHandlingMode->SetIntValue(ptrHandlingModeEntryVal);

        return true;
    }

    bool ConfigureStereoProcessing(INodeMap& nodeMapCamera, StereoParameters& stereoParameters)
    {
        bool result = true;

        // populate stereoParameters.disparityScaleFactor from the setting in the camera
        result = result && GetDisparityScaleFactor(nodeMapCamera, stereoParameters.disparityScaleFactor);

        result = result && GetRefCameraFocalLength(nodeMapCamera, stereoParameters.focalLength);

        result = result && GetCameraBaseLine(nodeMapCamera, stereoParameters.baseline);

        result = result && GetRefCameraImageCenter(
                               nodeMapCamera, stereoParameters.fCurrentCentreRow, stereoParameters.fCurrentCentreCol);

        return result;
    }

    bool ConfigureAcquisition(
        CameraPtr pCam,
        StreamTransmitFlags& streamTransmitFlags,
        bool doSetTransmittedStreams,
        std::map<SpinStereo::CameraStreamType, ImagePtr>& imageMap)
    {
        bool result = true;

        // Retrieve GenICam nodemap
        INodeMap& nodeMapTLDevice = pCam->GetTLStreamNodeMap();

        // there are 3 nodeMaps (map to the 3 node maps in SpinView):
        // - nodeMapCamera - the nodemap for the camera
        // - nodeMapTLDevice - the nodemap for the TLDevice
        // - nodeMapStreamGrabber - the nodemap for the StreamGrabber
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        // Configure heartbeat for GEV camera
#ifdef _DEBUG
        result = result && DisableGVCPHeartbeat(pCam);
#else
        result = result && ResetGVCPHeartbeat(pCam);
#endif

        result = result && SetAcquisitionMode(nodeMapCamera);

        result = result && SetStreamBufferHandlingMode(nodeMapTLDevice);

        // Configure the camera streams Enable var, either on the camera from streamTransmitFlags,
        // or vice versa, depending on doSetTransmittedStreams
        // Initialize imageMap
        result =
            result && SpinStereo::ConfigureCameraStreams(pCam, imageMap, doSetTransmittedStreams, streamTransmitFlags);

        return result;
    }

    bool GetDisparityScaleFactor(INodeMap& nodeMap, float& disparityScaleFactor)
    {
        float scaleFactor;
        if (!GetCameraScaleFactor(nodeMap, scaleFactor))
        {
            cout << "Failed to get the scaleFactor." << endl << endl;
            return false;
        }

#define WORKAROUND_IMPROPER_RESOLUTION_FOR_SCALE_FACTOR
#ifdef WORKAROUND_IMPROPER_RESOLUTION_FOR_SCALE_FACTOR
        // Disable the workaround once the scaleFactor value is provided more accurately from the camera.

        // Ideally scaleFactor and disparityScaleFactor should be the same but the value from
        // the camera is served with lesser resolution.
        // Hence we calculate the proper value and store it into disparityScaleFactor

        // calculate closest integer disparityScaleFactor, from scaleFactor (float)
        // (2^disparityScaleFactor)=1/scaleFactor
        double numFractionalBitsAsDouble = -std::log2(scaleFactor);

        // Round numFractionalBitsAsDouble to the nearest integer
        unsigned int numFractionalBits = static_cast<unsigned int>(std::round(numFractionalBitsAsDouble));

        disparityScaleFactor = 1.0f / static_cast<int>(std::pow(2, numFractionalBits));
#else
        disparityScaleFactor = scaleFactor;
#endif

        return true;
    }

    bool GetCameraScaleFactor(INodeMap& nodeMap, float& scaleFactor)
    {
        if (!GetFloatValueFromNode(nodeMap, "Scan3dCoordinateScale", scaleFactor))
        {
            std::cerr << "Failed to get the scaleFactor from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetFloatValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, float& nodeVal)
    {
        CFloatPtr floatPtr = nodeMap.GetNode(nodeName);
        if (!IsReadable(floatPtr))
        {
            std::ostringstream msg;
            msg << nodeName << " is not readable" << endl;
            cerr << msg.str();
            return false;
        }
        nodeVal = static_cast<float>(floatPtr->GetValue());

        return true;
    }

    bool ConfigureCameraStreams(
        CameraPtr pCam,
        std::map<CameraStreamType, ImagePtr>& imageMap,
        bool doSetTransmittedStreams,
        StreamTransmitFlags& streamTransmitFlags)
    {
        if (doSetTransmittedStreams)
        {
            bool isStreamingOrigState = pCam->IsStreaming();

            // Stop the streaming if already streaming.
            if (isStreamingOrigState)
            {
                cout << "Stop the stream." << endl;
                pCam->EndAcquisition();
            }

            // Set the transmitted streams in the camera, from stereoParameters.
            if (!SetTransmittedCameraStreams(pCam, streamTransmitFlags))
            {
                std::cerr << "Failed to set the streams enable val." << std::endl;
                return false;
            }

            if (isStreamingOrigState)
            {
                cout << "Restart the stream." << endl;
                pCam->BeginAcquisition();
            }
        }
        else
        {
            // Get the transmitted streams from the camera, and update the streamTransmitFlags.
            if (!GetTransmittedCameraStreams(pCam, streamTransmitFlags))
            {
                std::cerr << "Failed to get the streams enable val." << std::endl;
                return false;
            }
        }

        return true;
    }

    bool SetSGBM_params(CameraPtr pCam, StereoParameters& camParameters)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        // ---------------------------------------------------------------------------
        // - TotalDisparity (a.k.a. numDisparities) (this parameter value is fixed to 255 due to FPGA constraints)

        // ---------------------------------------------------------------------------
        // Scan3dCoordinateOffset (a.k.a. MinDisparity)
        float minDisparity;
        if (!GetFloatValueFromNode(nodeMapCamera, "Scan3dCoordinateOffset", minDisparity))
        {
            std::cerr << "Failed to get the Scan3dCoordinateOffset parameter from the camera." << std::endl;
            return false;
        }

        // the value of minDisparityMaxVal (768) in the camera is derived from (1023 - 255 = 768)
        // 1023 - (2^10) 10 bit for storing the integer part of the disparity
        //        (altough the actual maximum disparity value from the camera is 256 (2^8, i.e. occupies 8 bits out of the 10)
        // 255 - TotalDisparity (a.k.a. numDisparities) - this value is fixed due to FPGA constraints

        // ---------------------------------------------------------------------------
        // Scan3dCoordinateOffset Max value
        float minDisparityMaxVal;
        if (!GetMaxFloatValueFromNode(nodeMapCamera, "Scan3dCoordinateOffset", minDisparityMaxVal))
        {
            std::cerr << "Failed to get the Scan3dCoordinateOffset max value from the camera." << std::endl;
            return false;
        }

        if (camParameters.minDisparity > minDisparityMaxVal)
        {
            cerr << "minDisparity (" << camParameters.minDisparity
                 << ") is bigger than the maximum possible value: " << minDisparityMaxVal
                 << ". Clamping the value to the maximum possible value." << endl;

            camParameters.minDisparity = minDisparityMaxVal;
        }

        if ((camParameters.minDisparity - minDisparity) > std::numeric_limits<double>::epsilon())
        {
            if (pCam->IsStreaming())
            {
                // The camera is streaming, and we need to updtae a parameter that requires to stop the stream so:
                // - stop the stream
                // - update the parameter
                // - restart the stream

                cout << "Stop the stream." << endl;
                pCam->EndAcquisition();

                // Set the minDisparity parameter
                if (!SetFloatValueToNode(nodeMapCamera, "Scan3dCoordinateOffset", camParameters.minDisparity))
                {
                    std::cerr << "Failed to set the Scan3dCoordinateOffset parameter to the camera." << std::endl;
                    return false;
                }

                // the camera was streaming before we made the changes so restart the stream
                cout << "Restart the stream." << endl;
                pCam->BeginAcquisition();
            }
            else
            {
                // Set the minDisparity parameter
                if (!SetFloatValueToNode(nodeMapCamera, "Scan3dCoordinateOffset", camParameters.minDisparity))
                {
                    std::cerr << "Failed to set the Scan3dCoordinateOffset parameter to the camera." << std::endl;
                    return false;
                }
            }
        }

        // ---------------------------------------------------------------------------
        // Uniqueness Ratio

        int64_t uniquenessRatio;
        if (!GetIntValueFromNode(nodeMapCamera, "UniquenessRatio", uniquenessRatio))
        {
            std::cerr << "Failed to get the Uniqueness Ratio parameter from the camera." << std::endl;
            return false;
        }

        if (camParameters.uniquenessRatio != uniquenessRatio)
        {
            if (!SetIntValueToNode(nodeMapCamera, "UniquenessRatio", camParameters.uniquenessRatio))
            {
                std::cerr << "Failed to set the Uniqueness Ratio parameter to the camera." << std::endl;
                return false;
            }
        }

        // ---------------------------------------------------------------------------
        // P1

        int64_t p1;
        if (!GetSGBM_P1(pCam, p1))
        {
            cerr << "Failed to get camera SGBM P1 parameter." << endl;
            return false;
        }

        if (camParameters.SGBMP1 != p1)
        {
            if (!SetIntValueToNode(nodeMapCamera, "SmallPenalty", camParameters.SGBMP1))
            {
                std::cerr << "Failed to set the SGBM P1 parameter to the camera." << std::endl;
                return false;
            }
        }

        // ---------------------------------------------------------------------------
        // P2

        int64_t p2;
        if (!GetSGBM_P2(pCam, p2))
        {
            cerr << "Failed to get camera SGBM P2 parameter." << endl;
            return false;
        }

        if (camParameters.SGBMP2 != p2)
        {
            if (!SetIntValueToNode(nodeMapCamera, "LargePenalty", camParameters.SGBMP2))
            {
                std::cerr << "Failed to set the SGBM P2 parameter to the camera." << std::endl;
                return false;
            }
        }

        return true;
    }

    bool CameraStreamTypeToString(CameraStreamType cameraStreamType, string& cameraStreamTypeAsStr)
    {
        switch (cameraStreamType)
        {
        case CameraStreamType_RAW_LEFT:
        {
            cameraStreamTypeAsStr = std::string("RAW_LEFT");
            break;
        }
        case CameraStreamType_RAW_RIGHT:
        {
            cameraStreamTypeAsStr = std::string("RAW_RIGHT");
            break;
        }
        case CameraStreamType_RECT_LEFT:
        {
            cameraStreamTypeAsStr = std::string("RECT_LEFT");
            break;
        }
        case CameraStreamType_RECT_RIGHT:
        {
            cameraStreamTypeAsStr = std::string("RECT_RIGHT");
            break;
        }
        case CameraStreamType_DISPARITY:
        {
            cameraStreamTypeAsStr = std::string("DISPARITY");
            break;
        }
        default:
        {
            std::ostringstream msg;
            msg << "Camera stream type is not supported: " << cameraStreamType << std::endl;
            cout << msg.str();
            return false;
        }
        }

        return true;
    }

    void PrintStreamIndexMap(const std::map<CameraStreamType, int>& streamIndexMap)
    {
        cout << "-------------------------- " << endl;
        cout << "Get stream indices: " << endl;

        // Print the stream indices
        cout << "streamIndexMap.size(): " << streamIndexMap.size() << endl;
        for (const auto& pair : streamIndexMap)
        {
            string cameraStreamTypeAsStr;
            CameraStreamTypeToString(pair.first, cameraStreamTypeAsStr);
            std::cout << "Camera stream type: " << cameraStreamTypeAsStr << ", Stream index: " << pair.second
                      << std::endl;
        }
        cout << "-------------------------- " << endl << endl;

        return;
    }

    bool SetStreamIndexMap(
        CameraPtr pCam,
        const StreamTransmitFlags& streamTransmitFlags,
        std::map<CameraStreamType, int>& streamIndexMap)
    {
        cout << "Set stream indices: " << endl;

        bool isStreamingOrigState = pCam->IsStreaming();

        if (!isStreamingOrigState)
        {
            // Start the streaming (this is needed to get the streams indices map)
            pCam->BeginAcquisition();
        }

        // Get the stream indices.
        // this function should be called AFTER the camera starts streaming
        // (stream indices are sorted out in firmware only after the camera starts streaming)
        // Firmware needs to find a solution to mark the node inreadable until the node is ready to be read
        // A temporary workaround is to wait for sometime until the ComponentDestination node is flushed on the firmware
        // side. When firmware resolves the problem, Spinnaker3d will need to handle the case of the node not being
        // readable (e.g. by waiting sometime and trying again??)
        int numMilliSecs = 1000;
        std::cout << "Set stream index map: Sleeping for " << numMilliSecs << ", milliseconds..." << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(numMilliSecs));

        string cameraStreamTypeAsStr;
        int streamIndex;
        streamIndexMap.clear();

        if (streamTransmitFlags.rawLeftTransmitEnabled)
        {
            // Get the stream index for CameraStreamType_RAW_LEFT
            if (!GetStreamIndex(pCam, CameraStreamType_RAW_LEFT, streamIndex))
            {
                CameraStreamTypeToString(CameraStreamType_RAW_LEFT, cameraStreamTypeAsStr);
                std::cerr << "Failed to get the stream index for stream: " << cameraStreamTypeAsStr << std::endl;
                return false;
            }
            streamIndexMap[CameraStreamType_RAW_LEFT] = streamIndex;
        }

        if (streamTransmitFlags.rawRightTransmitEnabled)
        {
            // Get the stream index for CameraStreamType_RAW_RIGHT
            if (!GetStreamIndex(pCam, CameraStreamType_RAW_RIGHT, streamIndex))
            {
                CameraStreamTypeToString(CameraStreamType_RAW_RIGHT, cameraStreamTypeAsStr);
                std::cerr << "Failed to get the stream index for stream: " << cameraStreamTypeAsStr << std::endl;
                return false;
            }
            streamIndexMap[CameraStreamType_RAW_RIGHT] = streamIndex;
        }

        if (streamTransmitFlags.rectLeftTransmitEnabled)
        {
            // Get the stream index for CameraStreamType_RECT_LEFT
            if (!GetStreamIndex(pCam, CameraStreamType_RECT_LEFT, streamIndex))
            {
                CameraStreamTypeToString(CameraStreamType_RECT_LEFT, cameraStreamTypeAsStr);
                std::cerr << "Failed to get the stream index for stream: " << cameraStreamTypeAsStr << std::endl;
                return false;
            }
            streamIndexMap[CameraStreamType_RECT_LEFT] = streamIndex;
        }

        if (streamTransmitFlags.rectRightTransmitEnabled)
        {
            // Get the stream index for CameraStreamType_RECT_RIGHT
            if (!GetStreamIndex(pCam, CameraStreamType_RECT_RIGHT, streamIndex))
            {
                CameraStreamTypeToString(CameraStreamType_RECT_RIGHT, cameraStreamTypeAsStr);
                std::cerr << "Failed to get the stream index for stream: " << cameraStreamTypeAsStr << std::endl;
                return false;
            }
            streamIndexMap[CameraStreamType_RECT_RIGHT] = streamIndex;
        }

        if (streamTransmitFlags.disparityTransmitEnabled)
        {
            // Get the stream index for CameraStreamType_DISPARITY
            if (!GetStreamIndex(pCam, CameraStreamType_DISPARITY, streamIndex))
            {
                CameraStreamTypeToString(CameraStreamType_DISPARITY, cameraStreamTypeAsStr);
                std::cerr << "Failed to get the stream index for stream: " << cameraStreamTypeAsStr << std::endl;
                return false;
            }
            streamIndexMap[CameraStreamType_DISPARITY] = streamIndex;
        }

#ifdef _DEBUG
        // print StreamIndexMap
        PrintStreamIndexMap(streamIndexMap);
#endif

        if (!isStreamingOrigState)
        {
            // The camera is streaming and its original streaming state was not streaming.
            // Restore the camera streaming to its original streaming state
            cout << "Restore camera streaming to its original streaming state. Stop the stream." << endl;
            pCam->EndAcquisition();
        }

        return true;
    }

    bool GetStreamIndex(CameraPtr pCam, CameraStreamType cameraStreamType, int& streamIndex)
    {
        SensorType sensorTypeEnum;
        StreamImageType streamImageType;
        string cameraStreamTypeAsStr;

        if (!CameraStreamTypeToString(cameraStreamType, cameraStreamTypeAsStr))
        {
            std::cerr << "Failed to get the name for the cameraStreamType." << std::endl;
            return false;
        }

        // Get stream index for cameraStreamType
        if (!GetSensorTypeAndStreamImageType(cameraStreamType, sensorTypeEnum, streamImageType))
        {
            std::cerr << "Failed to get the sensor type and stream image type." << std::endl;
            return false;
        }

        // Set Node SourceSelector.
        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        if (!SetEnumValueToNode(nodeMapCamera, "SourceSelector", sensorTypeEnum))
        {
            std::cerr << "Failed to set SourceSelector node" << std::endl;
            return false;
        }

        // Set Node ComponentSelector.
        if (!SetEnumValueToNode(nodeMapCamera, "ComponentSelector", streamImageType))
        {
            std::cerr << "Failed to set ComponentSelector node" << std::endl;
            return false;
        }

        // Get value for Node ComponentEnable.
        bool nodeValAsBool;
        if (!GetBooleanValueFromNode(nodeMapCamera, "ComponentEnable", nodeValAsBool))
        {
            std::cerr << "Failed to get ComponentEnable node." << std::endl;
            return false;
        }

        // Get value for Node ComponentDestination.
        int64_t nodeValAsInt;
        if (!GetEnumValueFromNode(nodeMapCamera, "ComponentDestination", nodeValAsInt))
        {
            std::cerr << "Failed to get ComponentDestination node." << std::endl;
            return false;
        }

        streamIndex = (int)nodeValAsInt;

        return true;
    }

    bool GetSensorTypeAndStreamImageType(
        CameraStreamType cameraStreamType,
        SensorType& sensorTypeEnum,
        StreamImageType& streamImageType)
    {
        switch (cameraStreamType)
        {
        case CameraStreamType_RAW_LEFT:
        {
            sensorTypeEnum = SensorType_LEFT;
            streamImageType = StreamImageType_RAW;
            break;
        }
        case CameraStreamType_RAW_RIGHT:
        {
            sensorTypeEnum = SensorType_RIGHT;
            streamImageType = StreamImageType_RAW;
            break;
        }
        case CameraStreamType_RECT_LEFT:
        {
            sensorTypeEnum = SensorType_LEFT;
            streamImageType = StreamImageType_RECT;
            break;
        }
        case CameraStreamType_RECT_RIGHT:
        {
            sensorTypeEnum = SensorType_RIGHT;
            streamImageType = StreamImageType_RECT;
            break;
        }
        case CameraStreamType_DISPARITY:
        {
            sensorTypeEnum = SensorType_LEFT;
            streamImageType = StreamImageType_DISPARITY;
            break;
        }
        default:
        {
            std::ostringstream msg;
            msg << "Camera stream type is not supported: " << cameraStreamType << std::endl;
            cerr << msg.str();
            return false;
        }
        }

        return true;
    }

    // Update the streamTransmitFlags from the camera transmitted streams
    bool GetTransmittedCameraStreams(CameraPtr pCam, StreamTransmitFlags& streamTransmitFlags)
    {
        // Set streamTransmitFlags.rawLeftTransmitEnabled based on the camera setting
        if (!GetStreamEnableVal(pCam, CameraStreamType_RAW_LEFT, streamTransmitFlags.rawLeftTransmitEnabled))
        {
            std::cerr << "Failed to get the RAW_LEFT enable val." << std::endl;
            return false;
        }

        // Set streamTransmitFlags.rawRightTransmitEnabled based on the camera setting
        if (!GetStreamEnableVal(pCam, CameraStreamType_RAW_RIGHT, streamTransmitFlags.rawRightTransmitEnabled))
        {
            std::cerr << "Failed to get the RAW_RIGHT enable val." << std::endl;
            return false;
        }

        // Set streamTransmitFlags.rectLeftTransmitEnabled based on the camera setting
        if (!GetStreamEnableVal(pCam, CameraStreamType_RECT_LEFT, streamTransmitFlags.rectLeftTransmitEnabled))
        {
            std::cerr << "Failed to get the RECT_LEFT enable val." << std::endl;
            return false;
        }

        // Set streamTransmitFlags.rectRightTransmitEnabled based on the camera setting
        if (!GetStreamEnableVal(pCam, CameraStreamType_RECT_RIGHT, streamTransmitFlags.rectRightTransmitEnabled))
        {
            std::cerr << "Failed to get the RECT_RIGHT enable val." << std::endl;
            return false;
        }

        // Set streamTransmitFlags.disparityTransmitEnabled based on the camera setting
        if (!GetStreamEnableVal(pCam, CameraStreamType_DISPARITY, streamTransmitFlags.disparityTransmitEnabled))
        {
            std::cerr << "Failed to get the DISPARITY enable val." << std::endl;
            return false;
        }

        return true;
    }

    // Set the transmitted streams in the camera from the streamTransmitFlags
    bool SetTransmittedCameraStreams(CameraPtr pCam, StreamTransmitFlags& streamTransmitFlags)
    {
        // Set camera RAW_LEFT transmit based on streamTransmitFlags.rawLeftTransmitEnabled
        if (!SetStreamEnableVal(
                pCam, CameraStreamType_RAW_LEFT, static_cast<int>(streamTransmitFlags.rawLeftTransmitEnabled)))
        {
            std::cerr << "Failed to set the RAW_LEFT stream enable val" << std::endl;
            return false;
        }

        // Set camera RAW_RIGHT transmit based on streamTransmitFlags.rawRightTransmitEnabled
        if (!SetStreamEnableVal(
                pCam, CameraStreamType_RAW_RIGHT, static_cast<int>(streamTransmitFlags.rawRightTransmitEnabled)))
        {
            std::cerr << "Failed to set the RAW_RIGHT stream enable val" << std::endl;
            return false;
        }

        // Set camera RECT_LEFT transmit based on streamTransmitFlags.rectLeftTransmitEnabled
        if (!SetStreamEnableVal(
                pCam, CameraStreamType_RECT_LEFT, static_cast<int>(streamTransmitFlags.rectLeftTransmitEnabled)))
        {
            std::cerr << "Failed to set the RECT_LEFT stream enable val" << std::endl;
            return false;
        }

        // Set camera RECT_RIGHT transmit based on streamTransmitFlags.rectRightTransmitEnabled
        if (!SetStreamEnableVal(
                pCam, CameraStreamType_RECT_RIGHT, static_cast<int>(streamTransmitFlags.rectRightTransmitEnabled)))
        {
            std::cerr << "Failed to set the RECT_RIGHT stream enable val" << std::endl;
            return false;
        }

        // Set camera DISPARITY transmit based on streamTransmitFlags.disparityTransmitEnabled
        if (!SetStreamEnableVal(
                pCam, CameraStreamType_DISPARITY, static_cast<int>(streamTransmitFlags.disparityTransmitEnabled)))
        {
            std::cerr << "Failed to set the DISPARITY stream enable val" << std::endl;
            return false;
        }

        return true;
    }

    bool SetCameraStreamsPixelFormat(CameraPtr pCam)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        // Set RAW_LEFT pixel format
        if (!SetStreamPixelFormat(pCam, CameraStreamType_RAW_LEFT))
        {
            std::cerr << "Failed to set the RAW_LEFT stream pixel format" << std::endl;
            return false;
        }

        // Set RAW_RIGHT pixel format
        if (!SetStreamPixelFormat(pCam, CameraStreamType_RAW_RIGHT))
        {
            std::cerr << "Failed to set the RAW_RIGHT stream pixel format" << std::endl;
            return false;
        }

        // Set RECT_LEFT pixel format
        if (!SetStreamPixelFormat(pCam, CameraStreamType_RECT_LEFT))
        {
            std::cerr << "Failed to set the RECT_LEFT stream pixel format" << std::endl;
            return false;
        }

        // Set RECT_RIGHT pixel format
        if (!SetStreamPixelFormat(pCam, CameraStreamType_RECT_RIGHT))
        {
            std::cerr << "Failed to set the RECT_RIGHT stream pixel format" << std::endl;
            return false;
        }

        // Set DISPARITY pixel format
        if (!SetStreamPixelFormat(pCam, CameraStreamType_DISPARITY))
        {
            std::cerr << "Failed to set the DISPARITY stream pixel format" << std::endl;
            return false;
        }

        return true;
    }

    bool EnableAutoExposure(CameraPtr pCam)
    {
        pCam->ExposureAuto.SetValue(ExposureAuto_Continuous);
        return true;
    }

    bool GetExposureTime(CameraPtr pCam, double& exposureTime)
    {
        exposureTime = pCam->ExposureTime.GetValue();
        return true;
    }

    bool GetBooleanValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, bool& nodeVal)
    {
        CBooleanPtr booleanPtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(booleanPtr))
        {
            std::ostringstream msg;
            msg << nodeName << " is not readable" << endl;
            cerr << msg.str();
            return false;
        }

        nodeVal = booleanPtr->GetValue();

        return true;
    }

    bool GetIntValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, int64_t& nodeVal)
    {
        CIntegerPtr intPtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(intPtr))
        {
            cout << nodeName << " is not readable" << endl;
            return false;
        }
        nodeVal = (intPtr->GetValue());

        return true;
    }

    bool GetMaxIntValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, int64_t& nodeVal)
    {
        CIntegerPtr intPtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(intPtr))
        {
            cout << nodeName << " is not readable" << endl;
            return false;
        }
        nodeVal = (intPtr->GetMax());

        return true;
    }

    bool GetMaxFloatValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, float& nodeVal)
    {
        CFloatPtr floatPtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(floatPtr))
        {
            cout << nodeName << " is not readable" << endl;
            return false;
        }
        nodeVal = static_cast<float>(floatPtr->GetMax());

        return true;
    }

    bool GetEnumValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, int64_t& enumVal)
    {
        CEnumerationPtr cEnumerationPtr = nodeMap.GetNode(nodeName);
        CNodePtr pNode = (CNodePtr)cEnumerationPtr;

        if (IsReadable(pNode))
        {
            enumVal = ((CEnumerationPtr)pNode)->GetIntValue();
        }
        else
        {
            cout << "node: " << nodeName << ", is not readable." << endl;
            return false;
        }

        return true;
    }

    bool GetEnumAsStringValueFromNode(
        INodeMap& nodeMap,
        const gcstring& nodeName,
        int64_t& nodeValAsInt,
        gcstring& nodeValAsStr)
    {
        CEnumerationPtr nodePtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(nodePtr))
        {
            std::ostringstream msg;
            msg << "Unable to read " << nodeName << endl;
            cerr << msg.str();
            return false;
        }
        nodeValAsInt = nodePtr->GetIntValue();
        nodeValAsStr = nodePtr->GetEntry(nodeValAsInt)->ToString();

        return true;
    }

    bool GetStringValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, string& nodeVal)
    {
        CStringPtr nodePtr = nodeMap.GetNode(nodeName);
        if (!IsReadable(nodePtr))
        {
            std::ostringstream msg;
            msg << "Node: " << nodeName << " is not readable" << endl;
            cerr << msg.str();
            return false;
        }
        nodeVal = nodePtr->GetValue();

        return true;
    }

    bool SetExposureTime(CameraPtr pCam, double exposureTime)
    {
        pCam->ExposureAuto.SetValue(ExposureAuto_Off);
        if (exposureTime >= 0)
        {
            pCam->ExposureTime.SetValue(exposureTime);
        }

        return true;
    }

    bool EnableAutoGain(CameraPtr pCam)
    {
        pCam->GainAuto.SetValue(GainAuto_Continuous);
        return true;
    }

    bool GetGainValue(CameraPtr pCam, double& gainValue)
    {
        gainValue = pCam->Gain.GetValue();
        return true;
    }

    bool SetGainValue(CameraPtr pCam, double gainValue = 0)
    {
        pCam->GainAuto.SetValue(GainAuto_Off);
        if (gainValue >= 0)
        {
            pCam->Gain.SetValue(gainValue);
        }

        return true;
    }

    bool SetBooleanValueToNode(INodeMap& nodeMap, const gcstring& nodeName, bool isChecked)
    {
        CBooleanPtr booleanPtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(booleanPtr) || !IsWritable(booleanPtr))
        {
            std::ostringstream msg;
            msg << nodeName << " is not readable or writable  " << endl;
            cerr << msg.str();
            return false;
        }
        booleanPtr->SetValue(isChecked);

        return true;
    }

    bool SetIntValueToNode(INodeMap& nodeMap, const gcstring& nodeName, int64_t nodeVal)
    {
        CIntegerPtr intValue = nodeMap.GetNode(nodeName);

        if (!IsReadable(intValue) || !IsWritable(intValue))
        {
            std::ostringstream msg;
            msg << "Unable to read or write " << nodeName << " . Aborting..." << endl;
            cerr << msg.str();
            return false;
        }
        intValue->SetValue(nodeVal);

        return true;
    }

    bool SetFloatValueToNode(INodeMap& nodeMap, const gcstring& nodeName, float nodeVal)
    {
        CFloatPtr nodePtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(nodePtr) || !IsWritable(nodePtr))
        {
            std::ostringstream msg;
            msg << nodeName << "is not readable or writable" << endl;
            cerr << msg.str();
            return false;
        }
        nodePtr->SetValue(nodeVal);

        return true;
    }

    bool SetEnumValueToNode(INodeMap& nodeMap, const gcstring& nodeName, const int64_t enumVal, NodeMapType nodeMapType)
    {
        CEnumerationPtr cEnumerationPtr = nodeMap.GetNode(nodeName);
        CNodePtr pNode = (CNodePtr)cEnumerationPtr;

        if (IsWritable(((CEnumerationPtr)pNode)))
        {
            // set the value
            ((CEnumerationPtr)pNode)->SetIntValue(enumVal);
        }
        else
        {
            cout << "node: " << nodeName << ", is not writable." << endl;
            return false;
        }

        return true;
    }

    bool SetEnumAsStringValueToNode(INodeMap& nodeMap, const gcstring& nodeName, gcstring nodeEnumValAsStr)
    {
        CEnumerationPtr nodePtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(nodePtr) || !IsWritable(nodePtr))
        {
            cerr << "node: " << nodeName.c_str() << " is not readable or not writable" << endl;
            return false;
        }

        CEnumEntryPtr enumNodePtr = nodePtr->GetEntryByName(nodeEnumValAsStr);
        if (!IsReadable(enumNodePtr))
        {
            std::ostringstream msg;
            msg << "Node: " << nodeEnumValAsStr << " is not readable. Aborting..." << endl;
            cerr << msg.str();
            return false;
        }
        const int64_t enumNodeVal = enumNodePtr->GetValue();
        nodePtr->SetIntValue(enumNodeVal);

        return true;
    }

    bool SetStringValueToNode(INodeMap& nodeMap, const gcstring& nodeName, string& nodeVal)
    {
        CStringPtr nodePtr = nodeMap.GetNode(nodeName);

        if (!IsReadable(nodePtr) || !IsWritable(nodePtr))
        {
            std::ostringstream msg;
            msg << "Node: " << nodeName << "is not readable or writable" << endl;
            cerr << msg.str();
            return false;
        }
        nodePtr->SetValue(nodeVal.c_str());

        return true;
    }

    bool GetNextImageGrp(
        CameraPtr pCam,
        const StreamTransmitFlags& streamTransmitFlags,
        const std::map<CameraStreamType, int>& streamIndexMap,
        uint64_t timeoutInMilliSecs,
        std::map<CameraStreamType, ImagePtr>& imageMap)
    {
        try
        {
            // Get the next image group within a timeout period
            ImageList imageList = pCam->GetNextImageSync(timeoutInMilliSecs);
            bool isImageListIncomplete = false;

            if (streamTransmitFlags.rawLeftTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_LEFT);
                isImageListIncomplete = isImageListIncomplete || imageList[streamIndex]->IsIncomplete();
            }
            if (streamTransmitFlags.rawRightTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_RIGHT);
                isImageListIncomplete = isImageListIncomplete || imageList[streamIndex]->IsIncomplete();
            }
            if (streamTransmitFlags.rectLeftTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_LEFT);
                isImageListIncomplete = isImageListIncomplete || imageList[streamIndex]->IsIncomplete();
            }
            if (streamTransmitFlags.rectRightTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_RIGHT);
                isImageListIncomplete = isImageListIncomplete || imageList[streamIndex]->IsIncomplete();
            }
            if (streamTransmitFlags.disparityTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_DISPARITY);
                isImageListIncomplete = isImageListIncomplete || imageList[streamIndex]->IsIncomplete();
            }

            if (isImageListIncomplete)
            {
// #define LOG_VERBOSE_INCOMPLETE_IMAGES
#ifdef LOG_VERBOSE_INCOMPLETE_IMAGES
                std::ostringstream msg;
                msg << "Image List is incomplete: " << endl;
                if (streamTransmitFlags.rawLeftTransmitEnabled)
                {
                    const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_LEFT);
                    string cameraStreamTypeAsStr;
                    CameraStreamTypeToString(CameraStreamType_RAW_LEFT, cameraStreamTypeAsStr);
                    msg << "stream: " << cameraStreamTypeAsStr << " - "
                        << Image::GetImageStatusDescription(imageList[streamIndex]->GetImageStatus()) << endl;
                }
                if (streamTransmitFlags.rawRightTransmitEnabled)
                {
                    const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_RIGHT);
                    string cameraStreamTypeAsStr;
                    CameraStreamTypeToString(CameraStreamType_RAW_RIGHT, cameraStreamTypeAsStr);
                    msg << "stream: " << cameraStreamTypeAsStr << " - "
                        << Image::GetImageStatusDescription(imageList[streamIndex]->GetImageStatus()) << endl;
                }
                if (streamTransmitFlags.rectLeftTransmitEnabled)
                {
                    const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_LEFT);
                    string cameraStreamTypeAsStr;
                    CameraStreamTypeToString(CameraStreamType_RECT_LEFT, cameraStreamTypeAsStr);
                    msg << "stream: " << cameraStreamTypeAsStr << " - "
                        << Image::GetImageStatusDescription(imageList[streamIndex]->GetImageStatus()) << endl;
                }
                if (streamTransmitFlags.rectRightTransmitEnabled)
                {
                    const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_RIGHT);
                    string cameraStreamTypeAsStr;
                    CameraStreamTypeToString(CameraStreamType_RECT_RIGHT, cameraStreamTypeAsStr);
                    msg << "stream: " << cameraStreamTypeAsStr << " - "
                        << Image::GetImageStatusDescription(imageList[streamIndex]->GetImageStatus()) << endl;
                }
                if (streamTransmitFlags.disparityTransmitEnabled)
                {
                    const int& streamIndex = streamIndexMap.at(CameraStreamType_DISPARITY);
                    string cameraStreamTypeAsStr;
                    CameraStreamTypeToString(CameraStreamType_DISPARITY, cameraStreamTypeAsStr);
                    msg << "stream: " << cameraStreamTypeAsStr << " - "
                        << Image::GetImageStatusDescription(imageList[streamIndex]->GetImageStatus()) << endl;
                }

                msg << "..." << endl;
                cout << msg.str();
#else
                cout << "Image List is incomplete." << endl;
#endif
                imageList.Release();
                return false;
            }

            ImageProcessor processor;

            // Get the images from the camera
            if (streamTransmitFlags.rawLeftTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_LEFT);
                imageMap[CameraStreamType_RAW_LEFT] = imageList.GetByIndex(streamIndex);
            }
            if (streamTransmitFlags.rawRightTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RAW_RIGHT);
                imageMap[CameraStreamType_RAW_RIGHT] = imageList.GetByIndex(streamIndex);
            }

            if (streamTransmitFlags.rectLeftTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_LEFT);
                imageMap[CameraStreamType_RECT_LEFT] = imageList.GetByIndex(streamIndex);
            }

            if (streamTransmitFlags.rectRightTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_RECT_RIGHT);
                imageMap[CameraStreamType_RECT_RIGHT] = imageList.GetByIndex(streamIndex);
            }

            if (streamTransmitFlags.disparityTransmitEnabled)
            {
                const int& streamIndex = streamIndexMap.at(CameraStreamType_DISPARITY);
                imageMap[CameraStreamType_DISPARITY] = imageList.GetByIndex(streamIndex);
            }

#if 0
            cout << "imageGroupCounter: " << imageGroupCounter << endl;
            if(imageGroupCounter++ % printEveryNFrames == 0)
            {
                // print every printEveryNFrames frames

                PrintStreamIndexMap(streamIndexMap);

                if(!PrintSGBM_params(pCam))
                {
                    cerr << "Failed to get camera disparity configuration parameters." << endl;
                    return false;
                }
            }
#endif

            imageList.Release();
            return true;
        }
        catch (Spinnaker::Exception& se)
        {
            cout << "Spinnaker error: " << se.what() << endl;
            return false;
        }
        catch (std::exception e)
        {
            // everything else
            std::cout << "Unhandled exception caught" << std::endl;
            std::cout << e.what() << std::endl;
            return false;
        }
    }

    bool GetMinDisparity(INodeMap& nodeMapCamera, float& minDisparity)
    {
        if (!GetFloatValueFromNode(nodeMapCamera, "Scan3dCoordinateOffset", minDisparity))
        {
            std::cerr << "Failed to get the min disparity parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetInvalidDisparityCheckFlag(INodeMap& nodeMapCamera, bool& doInvalidDisparityCheck)
    {
        if (!GetBooleanValueFromNode(nodeMapCamera, "Scan3dInvalidDataFlag", doInvalidDisparityCheck))
        {
            std::cerr << "Failed to get the Scan3dInvalidDataFlag parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetInvalidDisparityValue(INodeMap& nodeMap, float& invalidDisparityValue)
    {
        if (!GetFloatValueFromNode(nodeMap, "Scan3dInvalidDataValue", invalidDisparityValue))
        {
            std::cerr << "Failed to get the invalidDisparityValue from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetTotalDisparities(CameraPtr pCam, int64_t& totalDisparities)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        if (!GetIntValueFromNode(nodeMapCamera, "TotalDisparity", totalDisparities))
        {
            std::cerr << "Failed to get the number of disparities parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetSGBM_P1(CameraPtr pCam, int64_t& p1)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        if (!GetIntValueFromNode(nodeMapCamera, "SmallPenalty", p1))
        {
            std::cerr << "Failed to get the SGBM P1 parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetSGBM_P2(CameraPtr pCam, int64_t& p2)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        if (!GetIntValueFromNode(nodeMapCamera, "LargePenalty", p2))
        {
            std::cerr << "Failed to get the SGBM P2 parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetUniquenessRatio(CameraPtr pCam, int64_t& uniquenessRatio)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        if (!GetIntValueFromNode(nodeMapCamera, "UniquenessRatio", uniquenessRatio))
        {
            std::cerr << "Failed to get the SGBM Uniqueness Ratio parameter from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool SetStreamPixelFormat(CameraPtr pCam, CameraStreamType cameraStreamType)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        string cameraStreamTypeAsStr;
        CameraStreamTypeToString(cameraStreamType, cameraStreamTypeAsStr);

        // Set pixel format for camera stream cameraStreamType
        if (!SetStreamComponent(nodeMapCamera, cameraStreamType))
        {
            std::cerr << "Failed to set the stream component" << std::endl;
            return false;
        }

        std::string pixelFormatStr;
        PixelFormatEnums pixelFormatEnum;

        CEnumerationPtr ptrPixelFormat = nodeMapCamera.GetNode("PixelFormat");
        if (!IsReadable(ptrPixelFormat) || !IsWritable(ptrPixelFormat))
        {
            cerr << "node PixelFormat isNotReadableOrNotWritable. Aborting..." << endl;
            return false;
        }

        switch (cameraStreamType)
        {
        case CameraStreamType_RAW_LEFT:
        case CameraStreamType_RAW_RIGHT:
        case CameraStreamType_RECT_LEFT:
        case CameraStreamType_RECT_RIGHT:
        {
            pixelFormatStr = std::string("RGB8Packed");
            pixelFormatEnum = PixelFormat_RGB8Packed;
            break;
        }
        case CameraStreamType_DISPARITY:
        {
            // tester mode is required to set to pixelformat which is different than RGB8Packed.
            pixelFormatStr = std::string("Mono16");
            pixelFormatEnum = PixelFormat_Mono16;
            break;
        }
        default:
        {
            std::ostringstream msg;
            msg << "Camera stream type is not supported: " << cameraStreamType << std::endl;
            cerr << msg.str();
            return false;
        }
        }

        CEnumEntryPtr ptrPixelFormatNew = ptrPixelFormat->GetEntryByName(pixelFormatStr.c_str());
        if (!IsReadable(ptrPixelFormatNew))
        {
            std::ostringstream msg;
            msg << "node PixelFormat" << pixelFormatStr << " isNotReadable. Aborting..." << endl;
            cerr << msg.str();
            return false;
        }

        int64_t pixelFormatEnum_new = ptrPixelFormatNew->GetValue();
        if (!SetIntValueToNode(nodeMapCamera, "PixelFormat", pixelFormatEnum_new))
        {
            std::cerr << "Failed to set the PixelFormat parameter to the camera." << std::endl;
            return false;
        }

        // sanity check
        if ((pixelFormatEnum != PixelFormat_RGB8Packed) && (pixelFormatEnum != PixelFormat_Mono16))
        {
            cout << "Pixel format is invalid (should be RGB8Packed (for Raw, and Rect), or Mono16 (for Disparity))"
                 << endl;
            return false;
        }

        return true;
    }

    bool SetStreamComponent(INodeMap& nodeMapCamera, CameraStreamType cameraStreamType)
    {
        SensorType sensorTypeEnum;
        StreamImageType streamImageType;

        if (!GetSensorTypeAndStreamImageType(cameraStreamType, sensorTypeEnum, streamImageType))
        {
            std::cerr << "Failed to get the sensor type and stream image type." << std::endl;
            return false;
        }

        // Set Node SourceSelector
        if (!SetEnumValueToNode(nodeMapCamera, "SourceSelector", sensorTypeEnum))
        {
            std::cerr << "Failed to set SourceSelector node" << std::endl;
            return false;
        }

        // Set Node ComponentSelector
        if (!SetEnumValueToNode(nodeMapCamera, "ComponentSelector", streamImageType))
        {
            std::cerr << "Failed to set ComponentSelector node" << std::endl;
            return false;
        }

        return true;
    }

    bool GetStreamEnableVal(CameraPtr pCam, CameraStreamType cameraStreamType, bool& componentEnableVal)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        string cameraStreamTypeAsStr;
        CameraStreamTypeToString(cameraStreamType, cameraStreamTypeAsStr);

        // Get enable val for camera stream: cameraStreamType
        if (!SetStreamComponent(nodeMapCamera, cameraStreamType))
        {
            std::cerr << "Failed to set the stream component" << std::endl;
            return false;
        }

        // Get Node ComponentEnable
        if (!GetBooleanValueFromNode(nodeMapCamera, "ComponentEnable", componentEnableVal))
        {
            std::cerr << "Failed to get ComponentEnable node" << std::endl;
            return false;
        }

        return true;
    }

    bool SetStreamEnableVal(CameraPtr pCam, CameraStreamType cameraStreamType, bool componentEnableVal)
    {
        INodeMap& nodeMapCamera = pCam->GetNodeMap();

        string cameraStreamTypeAsStr;
        CameraStreamTypeToString(cameraStreamType, cameraStreamTypeAsStr);

        // Set the stream component
        if (!SetStreamComponent(nodeMapCamera, cameraStreamType))
        {
            std::cerr << "Failed to set the stream component" << std::endl;
            return false;
        }

#define DO_DISABLE_PACKET_RESEND
#ifdef DO_DISABLE_PACKET_RESEND
        // Disable packet resends
        pCam->TLStream.StreamPacketResendEnable.SetValue(false);
#else
#endif

        // Enable/disable the transmit of cameraStreamType
        if (!SetBooleanValueToNode(nodeMapCamera, "ComponentEnable", componentEnableVal))
        {
            std::cerr << "Failed to set ComponentEnable node" << std::endl;
            return false;
        }
        return true;
    }

    bool PrintCameraCalibrationParams(INodeMap& nodeMap)
    {
        cout << "Camera calibration parameters: " << endl;
        float baseline, scaleFactor, focalLength, centerRow, centerCol;
        if (!GetCameraBaseLine(nodeMap, baseline))
        {
            cerr << "Failed to read the baseline from the camera" << endl;
            return false;
        }
        cout << "baseline: " << baseline << endl;

        if (!GetDisparityScaleFactor(nodeMap, scaleFactor))
        {
            cerr << "Failed to read the scale factor for the disparity from the camera" << endl;
            return false;
        }
        cout << "scaleFactor after round-up: " << scaleFactor << endl;

        if (!GetRefCameraFocalLength(nodeMap, focalLength))
        {
            cerr << "Failed to get camera focal length." << endl;
            return false;
        }
        cout << "focal length: " << focalLength << endl;

        if (!GetRefCameraImageCenter(nodeMap, centerRow, centerCol))
        {
            cerr << "Failed to get camera image centers." << endl;
            return false;
        }
        cout << "image centers: " << centerRow << ", " << centerCol << endl;

        return true;
    }

    bool PrintSGBM_params(CameraPtr pCam)
    {
        cout << "SGBM params: " << endl;

        INodeMap& nodeMapCamera = pCam->GetNodeMap();
        float minDisparity;
        if (!GetMinDisparity(nodeMapCamera, minDisparity))
        {
            cerr << "Failed to get camera Min disparity parameter." << endl;
            return false;
        }
        cout << "minDisparity: " << minDisparity << endl;

        bool doInvalidDisparityCheck;
        if (!GetInvalidDisparityCheckFlag(nodeMapCamera, doInvalidDisparityCheck))
        {
            cerr << "Failed to get camera doInvalidDisparityCheck parameter." << endl;
            return false;
        }
        cout << "doInvalidDisparityCheck: " << doInvalidDisparityCheck << endl;

        float invalidDisparityValue;
        if (!GetInvalidDisparityValue(nodeMapCamera, invalidDisparityValue))
        {
            cerr << "Failed to get camera invalidDisparityValue parameter." << endl;
            return false;
        }
        cout << "invalidDisparityValue: " << invalidDisparityValue << endl;

        int64_t totalDisparities;
        if (!GetTotalDisparities(pCam, totalDisparities))
        {
            cerr << "Failed to get camera Total Disparity parameter." << endl;
            return false;
        }
        cout << "Total Disparities: " << totalDisparities << endl;

        int64_t p1;
        if (!GetSGBM_P1(pCam, p1))
        {
            cerr << "Failed to get camera SGBM P1 parameter." << endl;
            return false;
        }
        cout << "p1: " << p1 << endl;

        int64_t p2;
        if (!GetSGBM_P2(pCam, p2))
        {
            cerr << "Failed to get camera SGBM P2 parameter." << endl;
            return false;
        }
        cout << "p2: " << p2 << endl;

        int64_t uniquenessRatio;
        if (!GetUniquenessRatio(pCam, uniquenessRatio))
        {
            cerr << "Failed to get camera Uniqueness Ratio parameter." << endl;
            return false;
        }
        cout << "uniquenessRatio: " << uniquenessRatio << endl;

        return true;
    }

    bool GetRefCameraFocalLength(INodeMap& nodeMap, float& focalLength)
    {
        if (!GetFloatValueFromNode(nodeMap, "Scan3dFocalLength", focalLength))
        {
            std::cerr << "Failed to get the focalLength from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetRefCameraImageCenter(INodeMap& nodeMap, float& cameraRowCenter, float& cameraColumnCenter)
    {
        if (!GetFloatValueFromNode(nodeMap, "Scan3dPrincipalPointV", cameraRowCenter))
        {
            std::cerr << "Failed to get the cameraRowCenter from the camera." << std::endl;
            return false;
        }

        if (!GetFloatValueFromNode(nodeMap, "Scan3dPrincipalPointU", cameraColumnCenter))
        {
            std::cerr << "Failed to get the cameraColumnCenter from the camera." << std::endl;
            return false;
        }

        return true;
    }

    bool GetCameraBaseLine(INodeMap& nodeMap, float& baseline)
    {
        if (!GetFloatValueFromNode(nodeMap, "Scan3dBaseline", baseline))
        {
            std::cerr << "Failed to get the baseline from the camera." << std::endl;
            return false;
        }
        return true;
    }

    bool GetCameraTemperature(double& temperature)
    {
        temperature = -273.0;
        return true;
    }

    bool PrintDeviceInfo(INodeMap& nodeMap)
    {
        bool result = true;
        cout << endl << "*** DEVICE INFORMATION ***" << endl;

        try
        {
            FeatureList_t features;
            const CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
            if (IsReadable(category))
            {
                category->GetFeatures(features);

                for (auto it = features.begin(); it != features.end(); ++it)
                {
                    const CNodePtr pfeatureNode = *it;
                    cout << pfeatureNode->GetName() << " : ";
                    CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
                    cout
                        << (IsReadable(pValue) ? pValue->ToString()
                                               : (Spinnaker::GenICam::gcstring) "Node not readable");
                    cout << endl;
                }
            }
            else
            {
                cout << "Device control information not available." << endl;
            }
        }
        catch (Spinnaker::Exception& e)
        {
            cout << "Error: " << e.what() << endl;
            result = false;
        }

        return result;
    }

    bool EnableAutoWhiteBalance(CameraPtr pCam)
    {

        pCam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);
        return true;
    }

    bool SetAutoWhiteBalance(CameraPtr pCam, double redVal, double blueVal)
    {
        pCam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Off);

        pCam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
        pCam->BalanceRatio.SetValue(redVal);

        pCam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
        pCam->BalanceRatio.SetValue(blueVal);

        return true;
    }

} // namespace SpinStereo