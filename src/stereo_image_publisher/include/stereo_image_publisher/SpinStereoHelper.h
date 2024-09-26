#ifndef STEREO_HELPER_H
#define STEREO_HELPER_H

/**
 * @file SpinStereoHelper.h
 * @brief Header file that includes definitions for the SpinStereoHelper namespace functions.
 */

/**
 * @defgroup camera_control Camera Control
 * @brief Functions and classes for controlling the camera.
 */

/**
 * @defgroup stereo_control Stereo Control
 * @brief Functions and classes for controlling stereo-specific parameters.
 */

#pragma once
#include "Spinnaker.h"
#include "StereoParameters.h"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace SpinStereo
{

    typedef enum NodeMapType { NodeMapType_Camera = 0, NodeMapType_TLStream = 1 } NodeMapType;

    typedef enum SensorType { SensorType_LEFT = 0, SensorType_RIGHT = 1 } SensorType;

    // the enum values are explicitly indexed to match the firmware indexing
    typedef enum StreamImageType {
        StreamImageType_RAW = 0,
        StreamImageType_RECT = 1,
        StreamImageType_DISPARITY = 2
    } StreamImageType;

    typedef enum CameraStreamType {
        CameraStreamType_RAW_LEFT,
        CameraStreamType_RAW_RIGHT,
        CameraStreamType_RECT_LEFT,
        CameraStreamType_RECT_RIGHT,
        CameraStreamType_DISPARITY
    } CameraStreamType;

    bool GetDisparityScaleFactor(INodeMap& nodeMap, float& disparityScaleFactor);

    /**
     * @brief Retrieves the disparity scale factor (the factor to multiply to get from
     * integer value to sub-pixel (floating point) disparity).
     * @param scaleFactor Variable to store the scale factor for the disparity.
     * @return True if successful, false otherwise.
     */
    bool GetCameraScaleFactor(INodeMap& nodeMap, float& scaleFactor);

    /**
     * @brief Configures the camera streams.
     * @param doSetTransmittedStreams Flag to set stream enable variable.
     * @return True if configuration is successful, false otherwise.
     * @ingroup camera_control
     */
    bool ConfigureCameraStreams(
        CameraPtr pCam,
        std::map<CameraStreamType, ImagePtr>& imageMap,
        bool doSetTransmittedStreams,
        StereoParameters& stereoParameters);

    /**
     * @brief Gets a float value from a specified node.
     * @param nodeName Node name.
     * @param nodeVal Reference to store the retrieved value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetFloatValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, float& nodeVal);

    /**
     * @brief Prints the content of stream index map.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    void PrintStreamIndexMap(const std::map<CameraStreamType, int>& streamIndexMap);

    /**
     * @brief Sets the stream index map after the camera starts streaming.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetStreamIndexMap(
        CameraPtr pCam,
        const StereoParameters& stereoParameters,
        std::map<CameraStreamType, int>& streamIndexMap);

    /**
     * @brief Gets the stream index for a specific stream type after the camera starts streaming.
     * @param cameraStreamType Camera stream type.
     * @param streamIndex Stream index.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetStreamIndex(CameraPtr pCam, CameraStreamType cameraStreamType, int& streamIndex);

    /**
     * @brief Gets the sensor type and stream image type.
     * @param cameraStreamType Camera stream type.
     * @param sensorType Sensor type.
     * @param streamImageType Stream image type.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetSensorTypeAndStreamImageType(
        CameraStreamType cameraStreamType,
        SensorType& sensorType,
        StreamImageType& streamImageType);

    /**
     * @brief Updates the stereoParameters from the stream enable variable from the camera.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetTransmittedCameraStreams(CameraPtr pCam, StereoParameters& stereoParameters);

    /**
     * @brief Sets the stream enable variable on the camera from the stereoParameters.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetTransmittedCameraStreams(CameraPtr pCam, StereoParameters& stereoParameters);

    /**
     * @brief Sets the pixel format for all camera streams.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetCameraStreamsPixelFormat();

    /**
     * @brief Enables automatic exposure.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool EnableAutoExposure(CameraPtr pCam);

    /**
     * @brief Retrieves the current exposure time.
     * @param exposureTime Reference to store the exposure time.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetExposureTime(CameraPtr pCam, double& exposureTime);

    /**
     * @brief Gets a boolean value from a specified node.
     * @param nodeName Node name.
     * @param nodeVal Reference to store the retrieved value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetBooleanValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, bool& nodeVal);

    /**
     * @brief Gets an integer value from a specified node.
     * @param nodeName Node name.
     * @param nodeVal Reference to store the retrieved value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetIntValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, int& nodeVal);

    /**
     * @brief Gets an enumerated (int) value from a specified node.
     * @param nodeName Node name.
     * @param enumVal Reference to store the retrieved value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetEnumValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, int& enumVal);

    /**
     * @brief Gets an enumerated string value from a specified node.
     * @param nodeName Node name.
     * @param nodeValAsInt Reference to store the integer value of the node.
     * @param nodeValAsStr Reference to store the string value of the node.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetEnumAsStringValueFromNode(
        INodeMap& nodeMap,
        const gcstring& nodeName,
        int& nodeValAsInt,
        gcstring& nodeValAsStr);

    /**
     * @brief Gets a string value from a specified node.
     * @param nodeName Node name.
     * @param nodeVal Reference to store the retrieved value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetStringValueFromNode(INodeMap& nodeMap, const gcstring& nodeName, string& nodeVal);

    /**
     * @brief Sets a boolean value to a specified node.
     * @param nodeName Node name.
     * @param isChecked Value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetBooleanValueToNode(INodeMap& nodeMap, const gcstring& nodeName, bool isChecked);

    /**
     * @brief Sets an integer value to a specified node.
     * @param nodeName Node name.
     * @param nodeVal Value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetIntValueToNode(INodeMap& nodeMap, const gcstring& nodeName, int nodeVal);

    /**
     * @brief Sets a double value to a specified node.
     * @param nodeName Node name.
     * @param nodeVal Value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetDoubleValueToNode(INodeMap& nodeMap, const gcstring& nodeName, double nodeVal);

    /**
     * @brief Sets an enumerated value to a specified node.
     * @param nodeMap Reference to the node map.
     * @param nodeName Node name.
     * @param nodeVal Value to set.
     * @param nodeMapType Type of the node map.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetEnumValueToNode(
        INodeMap& nodeMap,
        const gcstring& nodeName,
        const int64_t nodeVal,
        NodeMapType nodeMapType = NodeMapType_Camera);

    /**
     * @brief Sets an enumerated string value to a specified node.
     * @param nodeName Node name.
     * @param nodeEnumValAsStr String value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetEnumAsStringValueToNode(
        INodeMap& nodeMap,
        const gcstring& nodeName,
        gcstring nodeEnumValAsStr);

    /**
     * @brief Sets a string value to a specified node.
     * @param nodeName Node name.
     * @param nodeVal Value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetStringValueToNode(INodeMap& nodeMap, const gcstring& nodeName, string& nodeVal);

    /**
     * @brief Retrieves the minDisparity parameter.
     * @param minDisparity Variable to store the min disparity parameter.
     * @return True if successful, false otherwise.
     */
    bool GetMinDisparity(CameraPtr pCam, int& minDisparity);

    /**
     * @brief Retrieves the number of disparities parameter.
     * @param totalDisparities Variable to store the number of disparities parameter.
     * @return True if successful, false otherwise.
     */
    bool GetTotalDisparities(CameraPtr pCam, int& totalDisparities);

    /**
     * @brief Retrieves the SGBM P1 parameter.
     * @param p1 Variable to store the SGBM P1 parameter.
     * @return True if successful, false otherwise.
     */
    bool GetSGBM_P1(CameraPtr pCam, int& p1);

    /**
     * @brief Retrieves the SGBM P2 parameter.
     * @param p2 Variable to store the SGBM P2 parameter.
     * @return True if successful, false otherwise.
     */
    bool GetSGBM_P2(CameraPtr pCam, int& p2);

    /**
     * @brief Retrieves the SGBM Uniqueness Ratio parameter.
     * @param uniquenessRatio Variable to store the SGBM Uniqueness Ratio parameter.
     * @return True if successful, false otherwise.
     */
    bool GetUniquenessRatio(CameraPtr pCam, int& uniquenessRatio);

    /**
     * @brief Converts CameraStreamType to string.
     * @param cameraStreamType Camera stream type.
     * @param cameraStreamTypeAsStr String representation of the camera stream type.
     * @return True if conversion is successful, false otherwise.
     */
    bool CameraStreamTypeToString(CameraStreamType cameraStreamType, string& cameraStreamTypeAsStr);

    /**
     * @brief Gets the stream enable value from a node map.
     * @param nodeMapCamera Reference to the node map.
     * @param cameraStreamType Camera stream type.
     * @param componentEnableVal Component enable value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetStreamEnableVal(
        CameraPtr pCam,
        CameraStreamType cameraStreamType,
        bool& componentEnableVal);

    /**
     * @brief Sets the stream component in the node map.
     * @param nodeMapCamera Reference to the node map.
     * @param cameraStreamType Camera stream type.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetStreamComponent(INodeMap& nodeMapCamera, CameraStreamType cameraStreamType);

    /**
     * @brief Sets the stream enable value in the node map.
     * @param nodeMapCamera Reference to the node map.
     * @param cameraStreamType Camera stream type.
     * @param componentSelectorDoEnableVal Component selector enable value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetStreamEnableVal(
        CameraPtr pCam,
        CameraStreamType cameraStreamType,
        bool componentSelectorDoEnableVal);

    /**
     * @brief Sets the stream pixel format in the node map.
     * @param nodeMapCamera Reference to the node map.
     * @param cameraStreamType Camera stream type.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetStreamPixelFormat(CameraPtr pCam, CameraStreamType cameraStreamType);

    /**
     * @brief Configures the GVCP heartbeat setting.
     * @param doEnable Enable or disable GVCP heartbeat.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool ConfigureGVCPHeartbeat(bool doEnable);

    /**
     * @brief Disables the GVCP heartbeat.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool DisableGVCPHeartbeat();

    /**
     * @brief Prints device information using a specific node map.
     * @param nodeMap Reference to the node map.
     * @return Status code of the operation.
     * @ingroup camera_control
     */
    bool PrintDeviceInfo(INodeMap& nodeMap);

    /**
     * @brief Resets the GVCP heartbeat to its default state.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool ResetGVCPHeartbeat();

    /**
     * @brief Sets the exposure time.
     * @param exposureTime Exposure time to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetExposureTime(double exposureTime);

    /**
     * @brief Enables automatic gain control.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool EnableAutoGain();

    /**
     * @brief Retrieves the current gain value.
     * @param gainValue Reference to store the gain value.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool GetGainValue(double& gainValue);

    /**
     * @brief Sets the gain value.
     * @param gainValue Gain value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetGainValue(double gainValue);

    /**
     * @brief Sets the Automatic White Balance (AWB) using specified red and blue values.
     * @param redValue The red balance value to set.
     * @param blueValue The blue balance value to set.
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool SetAutoWhiteBalance(double redValue, double blueValue);

    /**
     * @brief Enables Automatic White Balance (AWB).
     * @return True if successful, false otherwise.
     * @ingroup camera_control
     */
    bool EnableAutoWhiteBalance(CameraPtr pCam);

    // /**
    //  * @brief Retrieves the next group of images for processing.
    //  * @return True if successful, false otherwise.
    //  * @ingroup camera_control
    //  */
    bool GetNextImageGrp(
        CameraPtr pCam,
        const StereoParameters& stereoParameters,
        std::map<CameraStreamType, int>& streamIndexMap,
        std::map<CameraStreamType, ImagePtr>& imageMap);

    /**
     * @brief Prints the camera calibration parameters.
     * @return True if successful, false otherwise.
     * @ingroup stereo_control
     */
    bool PrintCameraCalibrationParams(INodeMap& nodeMap);

    /** @} */ // end of camera_control

    /** @addtogroup stereo_control
     * @{
     */

    /**
     * @brief Retrieves the camera SGBM related parameters.
     * @return True if successful, false otherwise.
     */
    bool PrintSGBM_params(CameraPtr pCam);

    /**
     * @brief Retrieves the focal length of the reference camera.
     * @param focalLength Variable to store the focal length.
     * @return True if successful, false otherwise.
     */
    bool GetRefCameraFocalLength(INodeMap& nodeMap, float& focalLength);

    /**
     * @brief Retrieves the image center coordinates of the reference camera in pixels.
     * @param cameraRowCenter Variable to store the row center (cy) of the camera image.
     * @param cameraColumnCenter Variable to store the column center (cx) of the camera image.
     * @return True if successful, false otherwise.
     */
    bool GetRefCameraImageCenter(
        INodeMap& nodeMap,
        float& cameraRowCenter,
        float& cameraColumnCenter);

    /**
     * @brief Retrieves the baseline distance between the stereo cameras in meters.
     * @param baseline Variable to store the baseline distance.
     * @return True if successful, false otherwise.
     */
    bool GetCameraBaseLine(INodeMap& nodeMap, float& baseline);

    /**
     * @brief Retrieves the temperature of the camera.
     * @param temperature Variable to store the camera temperature.
     * @return True if successful, false otherwise.
     */
    bool GetCameraTemperature(double& temperature);

    /**
     * @brief Sets up the Semi-Global Block Matching algorithm parameters.
     * @param camParameters Pointer to the camera parameters.
     * @return True if successful, false otherwise.
     */
    bool SetSGBM_params(CameraPtr pCam, StereoParameters& camParameters);

    /**
     * @brief Post-process the disparity image.
     * @param windowSize The kernel size for disparity post-processing.
     * @param speckleThreshold The Speckle threshold for post-processing disparity.
     * @param minDisparity The minimum disparity that is used while calculating disparity.
     * @param disparityScaleFactor The scale factor (due to the sub-pixel bits) for the disparity image.
     * @return The post-processed disparity image.
     */
    ImagePtr PostProcessDisparity(
        ImagePtr disparityImagePtr,
        unsigned int windowSize,
        unsigned int speckleThreshold,
        const int minDisparity,
        const float disparityScaleFactor);

} // namespace SpinStereo

/** @} */ // end of stereo_control

#endif // STEREO_HELPER_H
