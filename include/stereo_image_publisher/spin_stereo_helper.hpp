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

#ifndef STEREO_HELPER_H
#define STEREO_HELPER_H

/**
 * @file SpinStereoHelper.h
 * @brief Header file that includes definitions for the SpinStereoHelper
 * namespace functions.
 */

#pragma once
#include "Spinnaker.h"
#include "stereo_parameters.hpp"

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;

namespace SpinStereo {

typedef enum NodeMapType {
  NodeMapType_Camera = 0,
  NodeMapType_TLStream = 1
} NodeMapType;

/**
 * @brief Retrieves the serial number of the camera.
 * @param p_cam Pointer to the camera object.
 * @return Serial number as a gcstring.
 */
gcstring get_serial_number(CameraPtr p_cam);

/**
 * @brief Configures the GVCP heartbeat of the camera.
 * @param p_cam Pointer to the camera object.
 * @param enable_heartbeat Boolean to enable or disable the heartbeat.
 * @return True if the heartbeat configuration was successful, false otherwise.
 */
bool configure_GVCP_heartbeat(CameraPtr p_cam, bool enable_heartbeat);

/**
 * @brief Resets the GVCP heartbeat to its default state.
 * @param p_cam Pointer to the camera object.
 * @return True if the reset was successful, false otherwise.
 */
bool reset_GVCP_heartbeat(CameraPtr p_cam);

/**
 * @brief Disables the GVCP heartbeat for the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if disabling the heartbeat was successful, false otherwise.
 */
bool disable_GVCP_heartbeat(CameraPtr p_cam);

/**
 * @brief Disables packet resends on the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if successful, false otherwise.
 */
bool disablePacketResends(CameraPtr p_cam);

/**
 * @brief Sets the device link throughput limit to the maximum supported value.
 * @param p_cam Pointer to the camera object.
 * @return True if successful, false otherwise.
 */
bool set_device_link_throughput_to_max(CameraPtr p_cam);

/**
 * @brief Sets the device link throughput limit to the current value.
 * @param p_cam Pointer to the camera object.
 * @return True if successful, false otherwise.
 */
bool set_device_link_throughput_to_current(CameraPtr p_cam);

/**
 * @brief Configures the device link throughput settings for the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if successful, false otherwise.
 */
bool set_device_link_throughput(CameraPtr p_cam);

/**
 * @brief Enables or disables the acquisition frame rate on the camera.
 * @param p_cam Pointer to the camera object.
 * @param enable Boolean to enable or disable frame rate control.
 * @return True if successful, false otherwise.
 */
bool set_acquisition_frame_rate_enabled(CameraPtr p_cam, bool enable);

/**
 * @brief Sets the acquisition frame rate for the camera.
 * @param p_cam Pointer to the camera object.
 * @param frame_rate Desired frame rate value to set.
 * @return True if successful, false otherwise.
 */
bool set_frame_rate(CameraPtr p_cam, float frame_rate);

/**
 * @brief Retrieves the current frame rate of the camera.
 * @param p_cam Pointer to the camera object.
 * @param resulting_frame_rate Reference to store the retrieved frame rate.
 * @return True if successful, false otherwise.
 */
bool get_frame_rate(CameraPtr p_cam, float &resulting_frame_rate);

/**
 * @brief Configures the acquisition mode of the camera.
 * @param node_map Reference to the node map object containing the camera
 * settings.
 * @return True if successful, false otherwise.
 */
bool set_acquisition_mode(INodeMap &node_map);

/**
 * @brief Configures the stream buffer handling mode for the camera.
 * @param node_map_TL_device Reference to the node map object for the TL device.
 * @return True if successful, false otherwise.
 */
bool set_stream_buffer_handling_mode(INodeMap &node_map_TL_device);

/**
 * @brief Configures stereo-specific processing settings on the camera.
 * @param node_map_camera Reference to the camera's node map object.
 * @param camera_parameters Reference to the stereo parameters structure.
 * @return True if successful, false otherwise.
 */
bool configure_stereo_processing(INodeMap &node_map_camera,
                                 StereoCameraParameters &stereo_parameters);

/**
 * @brief Configures the camera for image acquisition based on the stream
 * transmit flags.
 * @param p_cam Pointer to the camera object.
 * @param stream_transmit_flags Reference to the stream transmit flags.
 * @return True if successful, false otherwise.
 */
bool configure_acquisition(CameraPtr p_cam,
                           StreamTransmitFlags &stream_transmit_flags);

/**
 * @brief Retrieves the disparity scale factor from the camera settings.
 * @param node_map Reference to the node map object.
 * @param disparityScaleFactor Reference to store the retrieved disparity scale
 * factor.
 * @return True if successful, false otherwise.
 */
bool GetDisparityScaleFactor(INodeMap &node_map, float &disparityScaleFactor);

/**
 * @brief Retrieves the disparity scale factor (the factor to multiply to get
 * from integer value to sub-pixel (floating point) disparity).
 * @param scale_factor Variable to store the scale factor for the disparity.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_coordinate_scale(INodeMap &node_map, float &scale_factor);

/**
 * @brief Configures the camera streams.
 * @param doSetTransmittedStreams Flag to set stream enable variable.
 * @return True if configuration is successful, false otherwise.
 */
bool configure_camera_streams(CameraPtr p_cam,
                              StreamTransmitFlags &stream_transmit_flags);

/**
 * @brief Gets a float value from a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved value.
 * @return True if successful, false otherwise.
 */
bool get_float_value_from_node(INodeMap &node_map, const gcstring &node_name,
                               float &node_val);

/**
 * @brief Gets the max float value for a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved max value.
 * @return True if successful, false otherwise.
 */
bool get_min_float_value_from_node(INodeMap &node_map,
                                   const gcstring &node_name, float &node_val);

/**
 * @brief Gets the min float value for a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved min value.
 * @return True if successful, false otherwise.
 */
bool get_max_float_value_from_node(INodeMap &node_map,
                                   const gcstring &node_name, float &node_val);

/**
 * @brief Updates the camera_parameters from the stream enable variable from the
 * camera.
 * @return True if successful, false otherwise.
 */
bool GetTransmittedCameraStreams(CameraPtr p_cam,
                                 StreamTransmitFlags &stream_transmit_flags);

/**
 * @brief Sets the stream enable variable on the camera from the
 * camera_parameters.
 * @return True if successful, false otherwise.
 */
bool SetTransmittedCameraStreams(CameraPtr p_cam,
                                 StreamTransmitFlags &stream_transmit_flags);

/**
 * @brief Sets the pixel format for all camera streams.
 * @return True if successful, false otherwise.
 */
bool SetCameraStreamsPixelFormat();

/**
 * @brief Enables automatic exposure.
 * @return True if successful, false otherwise.
 */
bool enable_auto_exposure(CameraPtr p_cam);

/**
 * @brief Disables automatic exposure.
 * @return True if successful, false otherwise.
 */
bool disable_auto_exposure(CameraPtr p_cam);

/**
 * @brief Retrieves the current exposure time.
 * @param exposure_time Reference to store the exposure time.
 * @return True if successful, false otherwise.
 */
bool get_exposure_time(CameraPtr p_cam, float &exposure_time);

/**
 * @brief Gets a boolean value from a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved value.
 * @return True if successful, false otherwise.
 */
bool get_boolean_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 bool &node_val);

/**
 * @brief Gets an integer value from a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved value.
 * @return True if successful, false otherwise.
 */
bool get_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                             int64_t &node_val);

/**
 * @brief Gets the max integer value for a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved max value.
 * @return True if successful, false otherwise.
 */
bool get_min_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 int64_t &node_val);

/**
 * @brief Gets the min integer value for a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved min value.
 * @return True if successful, false otherwise.
 */
bool get_max_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 int64_t &node_val);

/**
 * @brief Gets an enumerated (int) value from a specified node.
 * @param node_name Node name.
 * @param enum_val Reference to store the retrieved value.
 * @return True if successful, false otherwise.
 */
bool get_enum_value_from_node(INodeMap &node_map, const gcstring &node_name,
                              int64_t &enum_val);

/**
 * @brief Gets an enumerated string value from a specified node.
 * @param node_name Node name.
 * @param node_val_as_int Reference to store the integer value of the node.
 * @param node_val_as_str Reference to store the string value of the node.
 * @return True if successful, false otherwise.
 */
bool get_enum_as_string_value_from_node(INodeMap &node_map,
                                        const gcstring &node_name,
                                        int &node_val_as_int,
                                        gcstring &node_val_as_str);

/**
 * @brief Gets a string value from a specified node.
 * @param node_name Node name.
 * @param node_val Reference to store the retrieved value.
 * @return True if successful, false otherwise.
 */
bool get_string_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                string &node_val);

/**
 * @brief Sets a boolean value to a specified node.
 * @param node_name Node name.
 * @param is_checked Value to set.
 * @return True if successful, false otherwise.
 */
bool set_boolean_value_to_node(INodeMap &node_map, const gcstring &node_name,
                               bool is_checked);

/**
 * @brief Sets an integer value to a specified node.
 * @param node_name Node name.
 * @param node_val Value to set.
 * @return True if successful, false otherwise.
 */
bool set_int_value_to_node(INodeMap &node_map, const gcstring &node_name,
                           int64_t node_val);

/**
 * @brief Sets a float value to a specified node.
 * @param node_name Node name.
 * @param node_val Value to set.
 * @return True if successful, false otherwise.
 */
bool set_float_value_to_node(INodeMap &node_map, const gcstring &node_name,
                             float node_val);

/**
 * @brief Sets an enumerated value to a specified node.
 * @param node_map Reference to the node map.
 * @param node_name Node name.
 * @param node_val Value to set.
 * @param node_map_type Type of the node map.
 * @return True if successful, false otherwise.
 */
bool set_enum_value_to_node(INodeMap &node_map, const gcstring &node_name,
                            const int64_t node_val,
                            NodeMapType node_map_type = NodeMapType_Camera);

/**
 * @brief Sets an enumerated string value to a specified node.
 * @param node_name Node name.
 * @param node_enum_val_as_str String value to set.
 * @return True if successful, false otherwise.
 */
bool set_enum_as_string_value_to_node(INodeMap &node_map,
                                      const gcstring &node_name,
                                      gcstring node_enum_val_as_str);

/**
 * @brief Sets a string value to a specified node.
 * @param node_name Node name.
 * @param node_val Value to set.
 * @return True if successful, false otherwise.
 */
bool set_string_value_to_node(INodeMap &node_map, const gcstring &node_name,
                              string &node_val);

/**
 * @brief Retrieves the coordinate_offset parameter.
 * @param coordinate_offset Variable to store the min disparity parameter.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_coordinate_offset(INodeMap &node_map_camera,
                                  float &coordinate_offset);

/**
 * @brief Retrieves the scan3d_invalid_data_flag parameter.
 * @param scan3d_invalid_data_flag Variable to store the invalidDataFlag
 * parameter.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_invalid_data_flag(INodeMap &node_map_camera,
                                  bool &scan3d_invalid_data_flag);

/**
 * @brief Retrieves the scan3d_invalid_data_value parameter.
 * @param scan3d_invalid_data_value Variable to store the
 * scan3d_invalid_data_value parameter.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_invalid_data_value(INodeMap &node_map,
                                   float &scan3d_invalid_data_value);

/**
 * @brief Retrieves the number of disparities parameter.
 * @param total_disparity Variable to store the number of disparities parameter.
 * @return True if successful, false otherwise.
 */
bool get_total_disparity(CameraPtr p_cam, int64_t &total_disparity);

/**
 * @brief Retrieves the small penalty parameter.
 * @param small_penalty Variable to store the small penalty parameter.
 * @return True if successful, false otherwise.
 */
bool get_small_penalty(CameraPtr p_cam, int64_t &small_penalty);

/**
 * @brief Retrieves the large penalty parameter.
 * @param large_penalty Variable to store the large penalty parameter.
 * @return True if successful, false otherwise.
 */
bool get_large_penalty(CameraPtr p_cam, int64_t &large_penalty);

/**
 * @brief Retrieves the SGBM Uniqueness Ratio parameter.
 * @param uniqueness_ratio Variable to store the SGBM Uniqueness Ratio
 * parameter.
 * @return True if successful, false otherwise.
 */
bool get_uniqueness_ratio(CameraPtr p_cam, int64_t &uniqueness_ratio);

/**
 * @brief Checks if raw image transmission is enabled for Sensor1.
 * @param p_cam Pointer to the camera object.
 * @return True if raw image transmission for Sensor1 is enabled, false
 * otherwise.
 */
bool is_raw_sensor_1_transmit_enabled(CameraPtr p_cam);

/**
 * @brief Checks if raw image transmission is enabled for Sensor2.
 * @param p_cam Pointer to the camera object.
 * @return True if raw image transmission for Sensor2 is enabled, false
 * otherwise.
 */
bool is_raw_sensor_2_transmit_enabled(CameraPtr p_cam);

/**
 * @brief Checks if rectified image transmission is enabled for Sensor1.
 * @param p_cam Pointer to the camera object.
 * @return True if rectified image transmission for Sensor1 is enabled, false
 * otherwise.
 */
bool is_rectified_sensor_1_transmit_enabled(CameraPtr p_cam);

/**
 * @brief Checks if rectified image transmission is enabled for Sensor2.
 * @param p_cam Pointer to the camera object.
 * @return True if rectified image transmission for Sensor2 is enabled, false
 * otherwise.
 */
bool is_rectified_sensor_2_transmit_enabled(CameraPtr p_cam);

/**
 * @brief Checks if disparity image transmission is enabled.
 * @param p_cam Pointer to the camera object.
 * @return True if disparity image transmission is enabled, false otherwise.
 */
bool is_disparity_transmit_enabled(CameraPtr p_cam);

/**
 * @brief Sets the Scan Coordinate Offset parameter in the camera.
 * @param p_cam Pointer to the camera object.
 * @param scan3d_coordinate_offset The new Scan Coordinate Offset value.
 * @return True if successful, false otherwise.
 */
bool set_scan3d_coordinate_offset(CameraPtr p_cam,
                                  float scan3d_coordinate_offset);

/**
 * @brief Sets the Uniqueness Ratio parameter in the camera.
 * @param p_cam Pointer to the camera object.
 * @param uniqueness_ratio The new Uniqueness Ratio value.
 * @return True if successful, false otherwise.
 */
bool set_uniqueness_ratio(CameraPtr p_cam, int64_t uniqueness_ratio);

/**
 * @brief Sets the Small Penalty parameter in the camera.
 * @param p_cam Pointer to the camera object.
 * @param small_penalty The new Small Penalty value.
 * @return True if successful, false otherwise.
 */
bool set_small_penalty(CameraPtr p_cam, int64_t small_penalty);

/**
 * @brief Sets the Large Penalty parameter in the camera.
 * @param p_cam Pointer to the camera object.
 * @param large_penalty The new Large Penalty value.
 * @return True if successful, false otherwise.
 */
bool set_large_penalty(CameraPtr p_cam, int64_t large_penalty);

/**
 * @brief Configures the GVCP heartbeat setting.
 * @param doEnable Enable or disable GVCP heartbeat.
 * @return True if successful, false otherwise.
 */
bool configure_GVCP_heartbeat(bool doEnable);

/**
 * @brief Disables the GVCP heartbeat.
 * @return True if successful, false otherwise.
 */
bool disable_GVCP_heartbeat();

/**
 * @brief Prints device information using a specific node map.
 * @param node_map Reference to the node map.
 * @return Status code of the operation.
 */
bool print_device_info(INodeMap &node_map);

/**
 * @brief Resets the GVCP heartbeat to its default state.
 * @return True if successful, false otherwise.
 */
bool reset_GVCP_heartbeat();

/**
 * @brief Sets the exposure time.
 * @param exposure_time Exposure time to set.
 * @return True if successful, false otherwise.
 */
bool set_exposure_time(CameraPtr p_cam, float exposure_time);

/**
 * @brief Enables automatic gain control.
 * @return True if successful, false otherwise.
 */
bool enable_auto_gain(CameraPtr p_cam);

/**
 * @brief Disable automatic gain control.
 * @return True if successful, false otherwise.
 */
bool disable_auto_gain(CameraPtr p_cam);

/**
 * @brief Retrieves the current gain value.
 * @param gain_value Reference to store the gain value.
 * @return True if successful, false otherwise.
 */
bool get_gain_value(CameraPtr p_cam, float &gain_value);

/**
 * @brief Sets the gain value.
 * @param gain_value Gain value to set.
 * @return True if successful, false otherwise.
 */
bool set_gain_value(CameraPtr p_cam, float gain_value);

/**
 * @brief Sets the Automatic White Balance (AWB) using specified red and blue
 * values.
 * @param redValue The red balance value to set.
 * @param blueValue The blue balance value to set.
 * @return True if successful, false otherwise.
 */
bool set_auto_white_balance(CameraPtr p_cam, float redValue, float blueValue);

/**
 * @brief Enables Automatic White Balance (AWB).
 * @return True if successful, false otherwise.
 */
bool enable_auto_white_balance(CameraPtr p_cam);

/**
 * @brief Retrieves the next group of images for processing.
 * @return True if successful, false otherwise.
 */
bool get_next_image_set(CameraPtr p_cam,
                        const StreamTransmitFlags &stream_transmit_flags,
                        uint64_t timeout_milli_secs, ImageList &image_list);

/**
 * @brief Prints the camera calibration parameters.
 * @return True if successful, false otherwise.
 */
bool print_camera_calibration_params(INodeMap &node_map);

/**
 * @brief Retrieves the camera SGBM related parameters.
 * @return True if successful, false otherwise.
 */
bool print_SGBM_params(CameraPtr p_cam);

/**
 * @brief Retrieves the focal length of the reference camera.
 * @param focalLength Variable to store the focal length.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_focal_length(INodeMap &node_map, float &focalLength);

/**
 * @brief Retrieves the image center coordinates of the reference camera in
 * pixels.
 * @param principalPointV Variable to store the row center (cy) of the camera
 * image.
 * @param principalPointU Variable to store the column center (cx) of the camera
 * image.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_principal_point(INodeMap &node_map, float &principalPointV,
                                float &principalPointU);

/**
 * @brief Retrieves the baseline distance between the stereo cameras in meters.
 * @param baseline Variable to store the baseline distance.
 * @return True if successful, false otherwise.
 */
bool get_scan3d_baseline(INodeMap &node_map, float &baseline);

/**
 * @brief Checks if the Acquisition Frame Rate is enabled on the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if the frame rate is enabled, false otherwise.
 */
bool is_acquisition_frame_rate_enabled(CameraPtr p_cam);

/**
 * @brief Checks if Auto Gain is enabled on the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if Auto Gain is enabled, false otherwise.
 */
bool is_auto_gain_enabled(CameraPtr p_cam);

/**
 * @brief Checks if Auto Exposure is enabled on the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if Auto Exposure is enabled, false otherwise.
 */
bool is_auto_exposure_enabled(CameraPtr p_cam);

/**
 * @brief Checks if Auto White Balance is enabled on the camera.
 * @param p_cam Pointer to the camera object.
 * @return True if Auto White Balance is enabled, false otherwise.
 */
bool is_auto_white_balance_enabled(CameraPtr p_cam);

}  // namespace SpinStereo

#endif  // STEREO_HELPER_H