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

#include "stereo_image_publisher/spin_stereo_helper.hpp"
#include <cmath>
#include <iostream>
#include <sstream>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "stereo_image_publisher/stereo_parameters.hpp"

using namespace std;

const unsigned int printEveryNFrames = 50;
unsigned int imageGroupCounter = 0;

namespace SpinStereo {

gcstring get_serial_number(CameraPtr p_cam) {
  INodeMap &node_map_TL_device = p_cam->GetTLDeviceNodeMap();

  gcstring device_serial_number("");
  CStringPtr ptr_string_serial =
      node_map_TL_device.GetNode("DeviceSerialNumber");

  if (IsReadable(ptr_string_serial)) {
    device_serial_number = ptr_string_serial->GetValue();
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Device serial number: %s", device_serial_number.c_str());
  } else {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "DeviceSerialNumber is not readable.");
  }

  return device_serial_number;
}

// Disables or enables heartbeat on GEV cameras so debugging does not incur
// timeout errors
bool configure_GVCP_heartbeat(CameraPtr p_cam, bool enable_heartbeat) {
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
  // happens, a camera power cycle will reset the heartbeat to its default
  // setting.

  // Retrieve TL device nodemap
  INodeMap &node_map_TL_device = p_cam->GetTLDeviceNodeMap();

  // Retrieve GenICam nodemap
  INodeMap &node_map = p_cam->GetNodeMap();

  CEnumerationPtr ptr_device_type = node_map_TL_device.GetNode("DeviceType");
  if (!IsReadable(ptr_device_type)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "DeviceType is not readable.");
    return false;
  }

  if (ptr_device_type->GetIntValue() != DeviceType_GigEVision) {
    return true;  // Not a GigE camera, no need to configure heartbeat.
  }

  if (enable_heartbeat) {
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Resetting heartbeat...");
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Disabling heartbeat...");
  }

  CBooleanPtr ptr_device_heartbeat =
      node_map.GetNode("GevGVCPHeartbeatDisable");
  if (!IsWritable(ptr_device_heartbeat)) {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "Unable to configure heartbeat. Continuing with execution as "
                "this may be non-fatal...");
  } else {
    ptr_device_heartbeat->SetValue(!enable_heartbeat);

    if (!enable_heartbeat) {
      RCLCPP_WARN(
          rclcpp::get_logger("stereo_image_publisher"),
          "WARNING: Heartbeat has been disabled for the rest of this run.\n"
          "         Heartbeat will be reset upon completion.\n"
          "         If the example is aborted unexpectedly, the camera may "
          "need to be power cycled.");
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                   "Heartbeat has been reset.");
    }
  }

  return true;
}

bool reset_GVCP_heartbeat(CameraPtr p_cam) {
  return configure_GVCP_heartbeat(p_cam, true);
}

bool disable_GVCP_heartbeat(CameraPtr p_cam) {
  return configure_GVCP_heartbeat(p_cam, false);
}

bool disable_packet_resends(CameraPtr p_cam) {
  try {
    // Retrieve TLStream nodemap (Transport Layer Stream)
    INodeMap &node_map_TL_stream = p_cam->GetTLStreamNodeMap();

    // Get the packet resend enable node
    CBooleanPtr ptr_packet_resend_enable =
        node_map_TL_stream.GetNode("StreamPacketResendEnable");

    if (IsWritable(ptr_packet_resend_enable)) {
      ptr_packet_resend_enable->SetValue(false);
      RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                   "Packet resends disabled.");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                  "Unable to disable packet resends.");
      return false;
    }

    return true;
  } catch (Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"), "Error: %s",
                 e.what());
    return false;
  }
}

bool set_device_link_throughput_to_max(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();

  CIntegerPtr ptr_max_throughput = node_map.GetNode("DeviceMaxThroughput");
  CIntegerPtr ptr_throughput_limit =
      node_map.GetNode("DeviceLinkThroughputLimit");

  if (!IsReadable(ptr_max_throughput)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read node DeviceMaxThroughput. Aborting...");
    return false;
  }

  if (!IsReadable(ptr_throughput_limit) || !IsWritable(ptr_throughput_limit)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write to node DeviceLinkThroughputLimit. "
                 "Aborting...");
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Current camera throughput: %ld",
               ptr_max_throughput->GetValue());

  if (ptr_throughput_limit->GetMin() > ptr_max_throughput->GetValue()) {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "DeviceLinkCurrentThroughput node minimum of: %ld is higher "
                "than desired throughput: %ld",
                ptr_throughput_limit->GetMin(), ptr_max_throughput->GetValue());
    ptr_throughput_limit->SetValue(ptr_throughput_limit->GetMin());
  } else if (ptr_throughput_limit->GetMax() < ptr_max_throughput->GetValue()) {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "DeviceLinkCurrentThroughput node maximum of: %ld is less than "
                "desired throughput: %ld",
                ptr_throughput_limit->GetMax(), ptr_max_throughput->GetValue());
    ptr_throughput_limit->SetValue(ptr_throughput_limit->GetMax());
  } else {
    ptr_throughput_limit->SetValue(ptr_max_throughput->GetValue());
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "DeviceLinkThroughputLimit set to: %ld",
               ptr_throughput_limit->GetValue());

  return true;
}

bool set_device_link_throughput_to_current(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();

  CIntegerPtr ptr_current_throughput =
      node_map.GetNode("DeviceLinkCurrentThroughput");
  CIntegerPtr ptr_throughput_limit =
      node_map.GetNode("DeviceLinkThroughputLimit");

  if (!IsReadable(ptr_current_throughput)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Unable to read node DeviceLinkCurrentThroughput. Aborting...");
    return false;
  }

  if (!IsReadable(ptr_throughput_limit) || !IsWritable(ptr_throughput_limit)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write to node DeviceLinkThroughputLimit. "
                 "Aborting...");
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Current camera throughput: %ld",
               ptr_current_throughput->GetValue());

  if (ptr_throughput_limit->GetMin() > ptr_current_throughput->GetValue()) {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "DeviceLinkCurrentThroughput node minimum of: %ld is higher "
                "than desired throughput: %ld",
                ptr_throughput_limit->GetMin(),
                ptr_current_throughput->GetValue());
    ptr_throughput_limit->SetValue(ptr_throughput_limit->GetMin());
  } else {
    ptr_throughput_limit->SetValue(ptr_current_throughput->GetValue());
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "DeviceLinkThroughputLimit set to: %ld",
               ptr_throughput_limit->GetValue());

  return true;
}

bool set_device_link_throughput(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();

  CIntegerPtr ptr_packet_size = node_map.GetNode("GevSCPSPacketSize");

  if (!IsReadable(ptr_packet_size) || !IsWritable(ptr_packet_size)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write packet size. Aborting...");
    return false;
  }

  const unsigned int max_packet_size = p_cam->DiscoverMaxPacketSize();
  const auto max_Gev_SCPS_packet_size =
      static_cast<unsigned int>(ptr_packet_size->GetMax());
  const auto desired_packet_size = max_Gev_SCPS_packet_size < max_packet_size
                                       ? max_Gev_SCPS_packet_size
                                       : max_packet_size;
  ptr_packet_size->SetValue(desired_packet_size);

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "PacketSize set to: %ld", ptr_packet_size->GetValue());

  CIntegerPtr ptr_current_throughput =
      node_map.GetNode("DeviceLinkCurrentThroughput");
  CIntegerPtr ptr_throughput_limit =
      node_map.GetNode("DeviceLinkThroughputLimit");

  if (!IsReadable(ptr_current_throughput)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Unable to read node DeviceLinkCurrentThroughput. Aborting...");
    return false;
  }

  if (!IsReadable(ptr_throughput_limit) || !IsWritable(ptr_throughput_limit)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write to node DeviceLinkThroughputLimit. "
                 "Aborting...");
    return false;
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Current camera throughput: %ld",
               ptr_current_throughput->GetValue());

  if (ptr_throughput_limit->GetMin() > ptr_current_throughput->GetValue()) {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "DeviceLinkCurrentThroughput node minimum of: %ld is higher "
                "than desired throughput: %ld",
                ptr_throughput_limit->GetMin(),
                ptr_current_throughput->GetValue());
    ptr_throughput_limit->SetValue(ptr_throughput_limit->GetMin());
  } else {
    ptr_throughput_limit->SetValue(ptr_current_throughput->GetValue());
  }

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "DeviceLinkThroughputLimit set to: %ld",
               ptr_throughput_limit->GetValue());

  return true;
}

bool set_acquisition_frame_rate_enabled(CameraPtr p_cam, bool enable) {
  try {
    RCLCPP_DEBUG(
        rclcpp::get_logger("stereo_image_publisher"),
        "Trying to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
    if (!set_device_link_throughput_to_max(p_cam)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("stereo_image_publisher"),
          "Unable to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
      return false;
    }

    INodeMap &node_map = p_cam->GetNodeMap();
    CBooleanPtr ptr_frame_rate_enable =
        node_map.GetNode("AcquisitionFrameRateEnable");
    if (IsWritable(ptr_frame_rate_enable)) {
      ptr_frame_rate_enable->SetValue(enable);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "Unable to set AcquisitionFrameRateEnable node.");
      return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Trying to set DeviceLinkThroughputLimit to "
                 "DeviceLinkCurrentThroughput");
    if (!set_device_link_throughput_to_current(p_cam)) {
      RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                  "Unable to set DeviceLinkThroughputLimit to "
                  "DeviceLinkCurrentThroughput. This might cause packet loss.");
    }
  } catch (Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"), "Error: %s",
                 e.what());
    return false;
  }
  return true;
}

bool set_frame_rate(CameraPtr p_cam, float frame_rate) {
  try {
    RCLCPP_DEBUG(
        rclcpp::get_logger("stereo_image_publisher"),
        "Trying to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
    if (!set_device_link_throughput_to_max(p_cam)) {
      RCLCPP_ERROR(
          rclcpp::get_logger("stereo_image_publisher"),
          "Unable to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
      return false;
    }

    INodeMap &node_map = p_cam->GetNodeMap();
    CBooleanPtr ptr_frame_rate_enable =
        node_map.GetNode("AcquisitionFrameRateEnable");
    if (!ptr_frame_rate_enable->GetValue()) {
      ptr_frame_rate_enable->SetValue(true);
      RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                   "Acquisition frame rate enabled to set the frame rate.");
    }

    CFloatPtr ptr_frame_rate = node_map.GetNode("AcquisitionFrameRate");
    if (IsWritable(ptr_frame_rate)) {
      if (frame_rate < ptr_frame_rate->GetMin()) {
        ptr_frame_rate->SetValue(ptr_frame_rate->GetMin());
        RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                    "Cannot set frame rate lower than %f FPS",
                    ptr_frame_rate->GetMin());
      } else if (frame_rate > ptr_frame_rate->GetMax()) {
        ptr_frame_rate->SetValue(ptr_frame_rate->GetMax());
        RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                    "Cannot set frame rate higher than %f FPS",
                    ptr_frame_rate->GetMax());
      } else {
        ptr_frame_rate->SetValue(frame_rate);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "Unable to set frame rate.");
      return false;
    }

    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Trying to set DeviceLinkThroughputLimit to "
                 "DeviceLinkCurrentThroughput");
    if (!set_device_link_throughput_to_current(p_cam)) {
      RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                  "Unable to set DeviceLinkThroughputLimit to "
                  "DeviceLinkCurrentThroughput. This might cause packet loss.");
    }

    return true;
  } catch (Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"), "Error: %s",
                 e.what());
    return false;
  }
}

bool get_frame_rate(CameraPtr p_cam, float &resulting_frame_rate) {
  try {
    INodeMap &node_map = p_cam->GetNodeMap();
    CFloatPtr ptr_resulting_frame_rate =
        node_map.GetNode("AcquisitionResultingFrameRate");

    if (IsReadable(ptr_resulting_frame_rate)) {
      resulting_frame_rate = ptr_resulting_frame_rate->GetValue();
      return true;
    } else {
      RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                  "Unable to read AcquisitionResultingFrameRate.");
      return false;
    }
  } catch (Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"), "Error: %s",
                 e.what());
    return false;
  }
}

bool set_acquisition_mode(INodeMap &node_map) {
  CEnumerationPtr ptr_acquisition_mode = node_map.GetNode("AcquisitionMode");
  if (!IsReadable(ptr_acquisition_mode) || !IsWritable(ptr_acquisition_mode)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to set acquisition mode to continuous (enum "
                 "retrieval). Aborting...");
    return false;
  }

  CEnumEntryPtr ptr_acquisition_mode_continuous =
      ptr_acquisition_mode->GetEntryByName("Continuous");
  if (!IsReadable(ptr_acquisition_mode_continuous)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to get or set acquisition mode to continuous (entry "
                 "retrieval). Aborting...");
    return false;
  }

  const int64_t acquisition_mode_continuous =
      ptr_acquisition_mode_continuous->GetValue();
  ptr_acquisition_mode->SetIntValue(acquisition_mode_continuous);

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Acquisition mode set to continuous.");

  return true;
}

bool set_stream_buffer_handling_mode(INodeMap &node_map_TL_device) {
  CEnumerationPtr ptr_handling_mode =
      node_map_TL_device.GetNode("StreamBufferHandlingMode");
  if (!IsReadable(ptr_handling_mode) || !IsWritable(ptr_handling_mode)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write to StreamBufferHandlingMode node. "
                 "Aborting...");
    return false;
  }

  CEnumEntryPtr ptr_handling_mode_entry =
      ptr_handling_mode->GetEntryByName("OldestFirst");
  if (!IsReadable(ptr_handling_mode_entry)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read ptr_handling_mode_entry node. Aborting...");
    return false;
  }

  int64_t ptr_handling_mode_entry_val = ptr_handling_mode_entry->GetValue();
  ptr_handling_mode->SetIntValue(ptr_handling_mode_entry_val);

  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "StreamBufferHandlingMode set to OldestFirst.");

  return true;
}

bool configure_stereo_processing(INodeMap &node_map_camera,
                                 StereoCameraParameters &stereo_parameters) {
  bool result = true;

  result = result && get_scan3d_focal_length(node_map_camera,
                                             stereo_parameters.focalLength);
  result = result &&
           get_scan3d_baseline(node_map_camera, stereo_parameters.baseline);
  result = result && get_scan3d_principal_point(
                         node_map_camera, stereo_parameters.principalPointV,
                         stereo_parameters.principalPointU);
  result =
      result && get_scan3d_coordinate_scale(
                    node_map_camera, stereo_parameters.disparityScaleFactor);
  result = result && get_scan3d_invalid_data_flag(
                         node_map_camera, stereo_parameters.invalidDataFlag);
  result = result && get_scan3d_invalid_data_value(
                         node_map_camera, stereo_parameters.invalidDataValue);

  if (result) {
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Stereo processing parameters configured successfully.");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to configure stereo processing parameters.");
  }

  return result;
}

bool configure_acquisition(CameraPtr p_cam,
                           StreamTransmitFlags &stream_transmit_flags) {
  bool result = true;

  INodeMap &node_map_TL_device = p_cam->GetTLStreamNodeMap();
  INodeMap &node_map_camera = p_cam->GetNodeMap();

  result = result && set_acquisition_mode(node_map_camera);
  result = result && set_stream_buffer_handling_mode(node_map_TL_device);

  if (!result) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to configure acquisition settings.");
    return false;
  }

  result = result && configure_camera_streams(p_cam, stream_transmit_flags);

  if (result) {
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Acquisition configured successfully.");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to configure camera streams.");
  }

  return result;
}

bool get_scan3d_coordinate_scale(INodeMap &node_map, float &scale_factor) {
  if (!get_float_value_from_node(node_map, "Scan3dCoordinateScale",
                                 scale_factor)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Failed to get the Scan3dCoordinateScale parameter from the camera.");
    return false;
  }
  return true;
}

bool get_float_value_from_node(INodeMap &node_map, const gcstring &node_name,
                               float &node_val) {
  CFloatPtr float_ptr = node_map.GetNode(node_name);
  if (!IsReadable(float_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable.", node_name.c_str());
    return false;
  }
  node_val = static_cast<float>(float_ptr->GetValue());
  return true;
}

bool configure_camera_streams(CameraPtr p_cam,
                              StreamTransmitFlags &stream_transmit_flags) {
  INodeMap &node_map = p_cam->GetNodeMap();

  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    if (!IsReadable(ptr_source_selector) ||
        !IsReadable(ptr_component_selector) ||
        !IsReadable(ptr_component_enable)) {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "Unable to read source selector or component selector.");
      return false;
    }

    GenApi::CEnumEntryPtr ptr_source_sensor_1 =
        ptr_source_selector->GetEntryByName("Sensor1");
    GenApi::CEnumEntryPtr ptr_source_sensor_2 =
        ptr_source_selector->GetEntryByName("Sensor2");
    GenApi::CEnumEntryPtr ptr_component_raw =
        ptr_component_selector->GetEntryByName("Raw");
    GenApi::CEnumEntryPtr ptr_component_rectified =
        ptr_component_selector->GetEntryByName("Rectified");
    GenApi::CEnumEntryPtr ptr_component_disparity =
        ptr_component_selector->GetEntryByName("Disparity");

    // Raw Sensor1
    ptr_source_selector->SetIntValue(ptr_source_sensor_1->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_raw->GetValue());
    ptr_component_enable->SetValue(
        stream_transmit_flags.raw_sensor_1_transmit_enabled);

    // Raw Sensor2
    ptr_source_selector->SetIntValue(ptr_source_sensor_2->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_raw->GetValue());
    ptr_component_enable->SetValue(
        stream_transmit_flags.raw_sensor_2_transmit_enabled);

    // Rectified Sensor1
    ptr_source_selector->SetIntValue(ptr_source_sensor_1->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_rectified->GetValue());
    ptr_component_enable->SetValue(
        stream_transmit_flags.rect_sensor_1_transmit_enabled);

    // Rectified Sensor2
    ptr_source_selector->SetIntValue(ptr_source_sensor_2->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_rectified->GetValue());
    ptr_component_enable->SetValue(
        stream_transmit_flags.rect_sensor_2_transmit_enabled);

    // Disparity Sensor1
    ptr_source_selector->SetIntValue(ptr_source_sensor_1->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_disparity->GetValue());
    ptr_component_enable->SetValue(
        stream_transmit_flags.disparity_transmit_enabled);
  } catch (Spinnaker::Exception &se) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to enable stereo source and components. Exception: %s",
                 se.what());
    return false;
  }

  return true;
}

bool is_raw_sensor_1_transmit_enabled(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();
  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    ptr_source_selector->SetIntValue(
        ptr_source_selector->GetEntryByName("Sensor1")->GetValue());
    ptr_component_selector->SetIntValue(
        ptr_component_selector->GetEntryByName("Raw")->GetValue());

    return ptr_component_enable->GetValue();
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check Raw Sensor1 transmission: %s", e.what());
    return false;
  }
}

bool is_raw_sensor_2_transmit_enabled(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();
  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    ptr_source_selector->SetIntValue(
        ptr_source_selector->GetEntryByName("Sensor2")->GetValue());
    ptr_component_selector->SetIntValue(
        ptr_component_selector->GetEntryByName("Raw")->GetValue());

    return ptr_component_enable->GetValue();
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check Raw Sensor2 transmission: %s", e.what());
    return false;
  }
}

bool is_rectified_sensor_1_transmit_enabled(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();
  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    ptr_source_selector->SetIntValue(
        ptr_source_selector->GetEntryByName("Sensor1")->GetValue());
    ptr_component_selector->SetIntValue(
        ptr_component_selector->GetEntryByName("Rectified")->GetValue());

    return ptr_component_enable->GetValue();
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check Rectified Sensor1 transmission: %s",
                 e.what());
    return false;
  }
}

bool is_rectified_sensor_2_transmit_enabled(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();
  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    ptr_source_selector->SetIntValue(
        ptr_source_selector->GetEntryByName("Sensor2")->GetValue());
    ptr_component_selector->SetIntValue(
        ptr_component_selector->GetEntryByName("Rectified")->GetValue());

    return ptr_component_enable->GetValue();
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check Rectified Sensor2 transmission: %s",
                 e.what());
    return false;
  }
}

bool is_disparity_transmit_enabled(CameraPtr p_cam) {
  INodeMap &node_map = p_cam->GetNodeMap();
  try {
    CEnumerationPtr ptr_source_selector = node_map.GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map.GetNode("ComponentSelector");
    CBooleanPtr ptr_component_enable = node_map.GetNode("ComponentEnable");

    ptr_source_selector->SetIntValue(
        ptr_source_selector->GetEntryByName("Sensor1")->GetValue());
    ptr_component_selector->SetIntValue(
        ptr_component_selector->GetEntryByName("Disparity")->GetValue());

    return ptr_component_enable->GetValue();
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check Disparity transmission: %s", e.what());
    return false;
  }
}

// Implementation of get functions
bool get_scan3d_coordinate_offset(CameraPtr p_cam, float &offset) {
  return get_float_value_from_node(p_cam->GetNodeMap(),
                                   "Scan3dCoordinateOffset", offset);
}

// Implementation of set functions
bool set_scan3d_coordinate_offset(CameraPtr p_cam,
                                  float scan3d_coordinate_offset) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  float current_offset, max_offset;

  if (!get_scan3d_coordinate_offset(p_cam, current_offset)) {
    RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                 "Failed to get Scan3dCoordinateOffset from the camera.");
    return false;
  }

  if (!get_max_float_value_from_node(p_cam->GetNodeMap(),
                                     "Scan3dCoordinateOffset", max_offset)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("sgbm_params"),
        "Failed to get maximum Scan3dCoordinateOffset from the camera.");
    return false;
  }

  if (scan3d_coordinate_offset > max_offset) {
    RCLCPP_WARN(rclcpp::get_logger("sgbm_params"),
                "Scan3dCoordinateOffset exceeds max value; clamping to %.2f.",
                max_offset);
    scan3d_coordinate_offset = max_offset;
  }

  if (fabs(scan3d_coordinate_offset - current_offset) >
      std::numeric_limits<double>::epsilon()) {
    if (!set_float_value_to_node(node_map_camera, "Scan3dCoordinateOffset",
                                 scan3d_coordinate_offset)) {
      RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                   "Failed to set Scan3dCoordinateOffset on camera.");
      return false;
    }
  }
  return true;
}

bool set_uniqueness_ratio(CameraPtr p_cam, int64_t uniqueness_ratio) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  int64_t current_ratio;

  if (!get_uniqueness_ratio(p_cam, current_ratio)) {
    RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                 "Failed to get UniquenessRatio from the camera.");
    return false;
  }

  if (uniqueness_ratio != current_ratio) {
    if (!set_int_value_to_node(node_map_camera, "UniquenessRatio",
                               uniqueness_ratio)) {
      RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                   "Failed to set UniquenessRatio on camera.");
      return false;
    }
  }
  return true;
}

bool set_small_penalty(CameraPtr p_cam, int64_t small_penalty) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  int64_t current_penalty;

  if (!get_small_penalty(p_cam, current_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                 "Failed to get SmallPenalty from the camera.");
    return false;
  }

  if (small_penalty != current_penalty) {
    if (!set_int_value_to_node(node_map_camera, "SmallPenalty",
                               small_penalty)) {
      RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                   "Failed to set SmallPenalty on camera.");
      return false;
    }
  }
  return true;
}

bool set_large_penalty(CameraPtr p_cam, int64_t large_penalty) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  int64_t current_penalty;

  if (!get_large_penalty(p_cam, current_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                 "Failed to get LargePenalty from the camera.");
    return false;
  }

  if (large_penalty != current_penalty) {
    if (!set_int_value_to_node(node_map_camera, "LargePenalty",
                               large_penalty)) {
      RCLCPP_ERROR(rclcpp::get_logger("sgbm_params"),
                   "Failed to set LargePenalty on camera.");
      return false;
    }
  }
  return true;
}

// bool SetSGBMParams(CameraPtr p_cam, StereoParameters& camParameters) {
//     if (!set_scan3d_coordinate_offset(p_cam, camParameters)) return false;
//     if (!set_uniqueness_ratio(p_cam, camParameters)) return false;
//     if (!set_small_penalty(p_cam, camParameters)) return false;
//     if (!set_large_penalty(p_cam, camParameters)) return false;
//     return true;
// }

bool enable_auto_exposure(CameraPtr p_cam) {
  p_cam->ExposureAuto.SetValue(ExposureAuto_Continuous);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Auto exposure enabled.");
  return true;
}

bool disable_auto_exposure(CameraPtr p_cam) {
  p_cam->ExposureAuto.SetValue(ExposureAuto_Off);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Auto exposure disabled.");
  return true;
}

bool get_exposure_time(CameraPtr p_cam, float &exposure_time) {
  exposure_time = p_cam->ExposureTime.GetValue();
  return true;
}

bool get_boolean_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 bool &node_val) {
  CBooleanPtr boolean_ptr = node_map.GetNode(node_name);

  if (!IsReadable(boolean_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = boolean_ptr->GetValue();
  return true;
}

bool get_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                             int64_t &node_val) {
  CIntegerPtr int_ptr = node_map.GetNode(node_name);

  if (!IsReadable(int_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = int_ptr->GetValue();
  return true;
}

bool get_min_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 int64_t &node_val) {
  CIntegerPtr int_ptr = node_map.GetNode(node_name);

  if (!IsReadable(int_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = int_ptr->GetMin();
  return true;
}

bool get_min_float_value_from_node(INodeMap &node_map,
                                   const gcstring &node_name, float &node_val) {
  CFloatPtr float_ptr = node_map.GetNode(node_name);

  if (!IsReadable(float_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = static_cast<float>(float_ptr->GetMin());
  return true;
}

bool get_max_int_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                 int64_t &node_val) {
  CIntegerPtr int_ptr = node_map.GetNode(node_name);

  if (!IsReadable(int_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = int_ptr->GetMax();
  return true;
}

bool get_max_float_value_from_node(INodeMap &node_map,
                                   const gcstring &node_name, float &node_val) {
  CFloatPtr float_ptr = node_map.GetNode(node_name);

  if (!IsReadable(float_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable", node_name.c_str());
    return false;
  }

  node_val = static_cast<float>(float_ptr->GetMax());
  return true;
}

bool get_enum_value_from_node(INodeMap &node_map, const gcstring &node_name,
                              int64_t &enum_val) {
  CEnumerationPtr c_enumeration_ptr = node_map.GetNode(node_name);
  CNodePtr p_node = (CNodePtr)c_enumeration_ptr;

  if (IsReadable(p_node)) {
    enum_val = c_enumeration_ptr->GetIntValue();
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node %s is not readable", node_name.c_str());
    return false;
  }

  return true;
}

bool get_enum_as_string_value_from_node(INodeMap &node_map,
                                        const gcstring &node_name,
                                        int64_t &node_val_as_int,
                                        gcstring &node_val_as_str) {
  CEnumerationPtr node_ptr = node_map.GetNode(node_name);

  if (!IsReadable(node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read %s", node_name.c_str());
    return false;
  }

  node_val_as_int = node_ptr->GetIntValue();
  node_val_as_str = node_ptr->GetEntry(node_val_as_int)->ToString();
  return true;
}

bool get_string_value_from_node(INodeMap &node_map, const gcstring &node_name,
                                std::string &node_val) {
  CStringPtr node_ptr = node_map.GetNode(node_name);
  if (!IsReadable(node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node: %s is not readable", node_name.c_str());
    return false;
  }

  node_val = node_ptr->GetValue();
  return true;
}

bool set_exposure_time(CameraPtr p_cam, float exposure_time) {
  p_cam->ExposureAuto.SetValue(ExposureAuto_Off);  // Disable auto exposure
  if (exposure_time >= 0) {
    p_cam->ExposureTime.SetValue(exposure_time);
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Exposure time set to: %f", exposure_time);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "Exposure time not set as the value provided is negative.");
  }

  return true;
}

bool enable_auto_gain(CameraPtr p_cam) {
  p_cam->GainAuto.SetValue(GainAuto_Continuous);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Auto gain enabled.");
  return true;
}

bool disable_auto_gain(CameraPtr p_cam) {
  p_cam->GainAuto.SetValue(GainAuto_Off);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Auto gain disabled.");
  return true;
}

bool get_gain_value(CameraPtr p_cam, float &gain_value) {
  gain_value = p_cam->Gain.GetValue();
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Current gain value: %f", gain_value);
  return true;
}

bool set_gain_value(CameraPtr p_cam, float gain_value = 0) {
  p_cam->GainAuto.SetValue(GainAuto_Off);  // Disable auto gain
  if (gain_value >= 0) {
    p_cam->Gain.SetValue(gain_value);
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Gain value set to: %f", gain_value);
  } else {
    RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                "Gain value not set as the value provided is negative.");
  }

  return true;
}

bool set_boolean_value_to_node(INodeMap &node_map, const gcstring &node_name,
                               bool is_checked) {
  CBooleanPtr boolean_ptr = node_map.GetNode(node_name);

  if (!IsReadable(boolean_ptr) || !IsWritable(boolean_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable or writable", node_name.c_str());
    return false;
  }

  boolean_ptr->SetValue(is_checked);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"), "Set %s to %s",
               node_name.c_str(), is_checked ? "true" : "false");
  return true;
}

bool set_int_value_to_node(INodeMap &node_map, const gcstring &node_name,
                           int64_t node_val) {
  CIntegerPtr int_value = node_map.GetNode(node_name);

  if (!IsReadable(int_value) || !IsWritable(int_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unable to read or write %s", node_name.c_str());
    return false;
  }

  int_value->SetValue(node_val);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"), "Set %s to %ld",
               node_name.c_str(), node_val);
  return true;
}

bool set_float_value_to_node(INodeMap &node_map, const gcstring &node_name,
                             float node_val) {
  CFloatPtr node_ptr = node_map.GetNode(node_name);

  if (!IsReadable(node_ptr) || !IsWritable(node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "%s is not readable or writable", node_name.c_str());
    return false;
  }

  node_ptr->SetValue(node_val);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"), "Set %s to %f",
               node_name.c_str(), node_val);
  return true;
}

bool set_enum_value_to_node(INodeMap &node_map, const gcstring &node_name,
                            const int64_t enum_val, NodeMapType node_map_type) {
  CEnumerationPtr c_enumeration_ptr = node_map.GetNode(node_name);
  CNodePtr p_node = (CNodePtr)c_enumeration_ptr;

  if (IsWritable(((CEnumerationPtr)p_node))) {
    ((CEnumerationPtr)p_node)->SetIntValue(enum_val);
    RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                 "Set enum %s to %ld", node_name.c_str(), enum_val);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node %s is not writable", node_name.c_str());
    return false;
  }

  return true;
}

bool set_enum_as_string_value_to_node(INodeMap &node_map,
                                      const gcstring &node_name,
                                      gcstring node_enum_val_as_str) {
  CEnumerationPtr node_ptr = node_map.GetNode(node_name);

  if (!IsReadable(node_ptr) || !IsWritable(node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node %s is not readable or writable", node_name.c_str());
    return false;
  }

  CEnumEntryPtr enum_node_ptr = node_ptr->GetEntryByName(node_enum_val_as_str);
  if (!IsReadable(enum_node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node %s is not readable", node_enum_val_as_str.c_str());
    return false;
  }

  const int64_t enumNodeVal = enum_node_ptr->GetValue();
  node_ptr->SetIntValue(enumNodeVal);
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Set enum %s to %s", node_name.c_str(),
               node_enum_val_as_str.c_str());

  return true;
}

bool set_string_value_to_node(INodeMap &node_map, const gcstring &node_name,
                              std::string &node_val) {
  CStringPtr node_ptr = node_map.GetNode(node_name);

  if (!IsReadable(node_ptr) || !IsWritable(node_ptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Node %s is not readable or writable", node_name.c_str());
    return false;
  }

  node_ptr->SetValue(node_val.c_str());
  RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
               "Set string %s to %s", node_name.c_str(), node_val.c_str());

  return true;
}

bool get_next_image_set(CameraPtr p_cam,
                        const StreamTransmitFlags &stream_transmit_flags,
                        uint64_t timeout_milli_secs, ImageList &image_list) {
  try {
    // Get the next image group within a timeout period
    image_list = p_cam->GetNextImageSync(timeout_milli_secs);

    bool is_image_list_incomplete = false;

    if (stream_transmit_flags.raw_sensor_1_transmit_enabled) {
      ImagePtr pImage =
          image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1);
      is_image_list_incomplete |=
          (pImage != nullptr ? pImage->IsIncomplete() : true);
    }

    if (stream_transmit_flags.raw_sensor_2_transmit_enabled) {
      ImagePtr pImage =
          image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2);
      is_image_list_incomplete |=
          (pImage != nullptr ? pImage->IsIncomplete() : true);
    }

    if (stream_transmit_flags.rect_sensor_1_transmit_enabled) {
      ImagePtr pImage = image_list.GetByPayloadType(
          SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);
      is_image_list_incomplete |=
          (pImage != nullptr ? pImage->IsIncomplete() : true);
    }

    if (stream_transmit_flags.rect_sensor_2_transmit_enabled) {
      ImagePtr pImage = image_list.GetByPayloadType(
          SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2);
      is_image_list_incomplete |=
          (pImage != nullptr ? pImage->IsIncomplete() : true);
    }

    if (stream_transmit_flags.disparity_transmit_enabled) {
      ImagePtr pImage = image_list.GetByPayloadType(
          SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
      is_image_list_incomplete |=
          (pImage != nullptr ? pImage->IsIncomplete() : true);
    }

    if (is_image_list_incomplete) {
      return false;
    }
    return true;
  } catch (const Spinnaker::Exception &se) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Spinnaker error: %s", se.what());
    return false;
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unhandled exception caught: %s", e.what());
    return false;
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Unknown error occurred.");
    return false;
  }
}

bool get_scan3d_coordinate_offset(INodeMap &node_map_camera,
                                  float &coordinate_offset) {
  if (!get_float_value_from_node(node_map_camera, "Scan3dCoordinateOffset",
                                 coordinate_offset)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Failed to get the Scan3dCoordinateOffset parameter from the camera.");
    return false;
  }
  return true;
}

bool get_scan3d_invalid_data_flag(INodeMap &node_map_camera,
                                  bool &scan3d_invalid_data_flag) {
  if (!get_boolean_value_from_node(node_map_camera, "Scan3dInvalidDataFlag",
                                   scan3d_invalid_data_flag)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Failed to get the Scan3dInvalidDataFlag parameter from the camera.");
    return false;
  }
  return true;
}

bool get_scan3d_invalid_data_value(INodeMap &node_map,
                                   float &scan3d_invalid_data_value) {
  if (!get_float_value_from_node(node_map, "Scan3dInvalidDataValue",
                                 scan3d_invalid_data_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the Scan3dInvalidDataValue from the camera.");
    return false;
  }
  return true;
}

bool get_total_disparity(CameraPtr p_cam, int64_t &total_disparity) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  if (!get_int_value_from_node(node_map_camera, "TotalDisparity",
                               total_disparity)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the TotalDisparity parameter from the camera.");
    return false;
  }
  return true;
}

bool get_small_penalty(CameraPtr p_cam, int64_t &small_penalty) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  if (!get_int_value_from_node(node_map_camera, "SmallPenalty",
                               small_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the SmallPenalty parameter from the camera.");
    return false;
  }
  return true;
}

bool get_large_penalty(CameraPtr p_cam, int64_t &large_penalty) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  if (!get_int_value_from_node(node_map_camera, "LargePenalty",
                               large_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the LargePenalty parameter from the camera.");
    return false;
  }
  return true;
}

bool get_uniqueness_ratio(CameraPtr p_cam, int64_t &uniqueness_ratio) {
  INodeMap &node_map_camera = p_cam->GetNodeMap();
  if (!get_int_value_from_node(node_map_camera, "UniquenessRatio",
                               uniqueness_ratio)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Failed to get the UniquenessRatio parameter from the camera.");
    return false;
  }
  return true;
}

bool print_camera_calibration_params(INodeMap &node_map) {
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Camera calibration parameters:");

  float baseline, scale_factor, focal_length, centerRow, centerCol;

  if (!get_scan3d_baseline(node_map, baseline)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to read the baseline from the camera");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "Baseline: %f",
              baseline);

  if (!get_scan3d_coordinate_scale(node_map, scale_factor)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("stereo_image_publisher"),
        "Failed to read the scale factor for the disparity from the camera");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Scale factor after round-up: %f", scale_factor);

  if (!get_scan3d_focal_length(node_map, focal_length)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera focal length.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "Focal length: %f",
              focal_length);

  if (!get_scan3d_principal_point(node_map, centerRow, centerCol)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera image centers.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Image centers: row = %f, col = %f", centerRow, centerCol);

  return true;
}

bool print_SGBM_params(CameraPtr p_cam) {
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "SGBM params:");

  INodeMap &node_map_camera = p_cam->GetNodeMap();
  float coordinate_offset;
  if (!get_scan3d_coordinate_offset(node_map_camera, coordinate_offset)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera coordinate_offset parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "CoordinateOffset: %f", coordinate_offset);

  bool scan3d_invalid_data_flag;
  if (!get_scan3d_invalid_data_flag(node_map_camera,
                                    scan3d_invalid_data_flag)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera scan3d_invalid_data_flag parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Scan3dInvalidDataFlag: %s",
              scan3d_invalid_data_flag ? "true" : "false");

  float scan3d_invalid_data_value;
  if (!get_scan3d_invalid_data_value(node_map_camera,
                                     scan3d_invalid_data_value)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera scan3d_invalid_data_value parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Scan3dInvalidDataValue: %f", scan3d_invalid_data_value);

  int64_t total_disparity;
  if (!get_total_disparity(p_cam, total_disparity)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera Total Disparity parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "Total Disparity: %ld", total_disparity);

  int64_t small_penalty;
  if (!get_small_penalty(p_cam, small_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera small penalty parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "SmallPenalty: %ld",
              small_penalty);

  int64_t large_penalty;
  if (!get_large_penalty(p_cam, large_penalty)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera large penalty parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "LargePenalty: %ld",
              large_penalty);

  int64_t uniqueness_ratio;
  if (!get_uniqueness_ratio(p_cam, uniqueness_ratio)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get camera Uniqueness Ratio parameter.");
    return false;
  }
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "UniquenessRatio: %ld", uniqueness_ratio);

  return true;
}

bool get_scan3d_focal_length(INodeMap &node_map, float &focal_length) {
  if (!get_float_value_from_node(node_map, "Scan3dFocalLength", focal_length)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the focal length from the camera.");
    return false;
  }
  return true;
}

bool get_scan3d_principal_point(INodeMap &node_map, float &principal_point_V,
                                float &principal_point_U) {
  if (!get_float_value_from_node(node_map, "Scan3dPrincipalPointV",
                                 principal_point_V)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the Scan3dPrincipalPointV from the camera.");
    return false;
  }

  if (!get_float_value_from_node(node_map, "Scan3dPrincipalPointU",
                                 principal_point_U)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the Scan3dPrincipalPointU from the camera.");
    return false;
  }

  return true;
}

bool get_scan3d_baseline(INodeMap &node_map, float &baseline) {
  if (!get_float_value_from_node(node_map, "Scan3dBaseline", baseline)) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to get the baseline from the camera.");
    return false;
  }
  return true;
}

bool print_device_info(INodeMap &node_map) {
  bool result = true;
  RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"),
              "*** DEVICE INFORMATION ***");

  try {
    FeatureList_t features;
    const CCategoryPtr category = node_map.GetNode("DeviceInformation");
    if (IsReadable(category)) {
      category->GetFeatures(features);

      for (auto it = features.begin(); it != features.end(); ++it) {
        const CNodePtr pfeatureNode = *it;
        CValuePtr pValue = static_cast<CValuePtr>(pfeatureNode);
        RCLCPP_INFO(rclcpp::get_logger("stereo_image_publisher"), "%s : %s",
                    pfeatureNode->GetName().c_str(),
                    (IsReadable(pValue) ? pValue->ToString().c_str()
                                        : "Node not readable"));
      }
    } else {
      RCLCPP_WARN(rclcpp::get_logger("stereo_image_publisher"),
                  "Device control information not available.");
    }
  } catch (Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"), "Error: %s",
                 e.what());
    result = false;
  }

  return result;
}

bool enable_auto_white_balance(CameraPtr p_cam) {
  p_cam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Continuous);
  return true;
}

bool disable_auto_white_balance(CameraPtr p_cam) {
  p_cam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Off);
  return true;
}

bool set_auto_white_balance(CameraPtr p_cam, float red_value,
                            float blue_value) {
  p_cam->BalanceWhiteAuto.SetValue(BalanceWhiteAuto_Off);

  p_cam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Red);
  p_cam->BalanceRatio.SetValue(red_value);

  p_cam->BalanceRatioSelector.SetValue(BalanceRatioSelector_Blue);
  p_cam->BalanceRatio.SetValue(blue_value);

  return true;
}
bool is_acquisition_frame_rate_enabled(CameraPtr p_cam) {
  try {
    INodeMap &node_map = p_cam->GetNodeMap();
    CBooleanPtr ptr_frame_rate_enable =
        node_map.GetNode("AcquisitionFrameRateEnable");
    if (IsReadable(ptr_frame_rate_enable)) {
      return ptr_frame_rate_enable->GetValue();
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "AcquisitionFrameRateEnable node is not readable.");
      return false;
    }
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check if frame rate is enabled: %s", e.what());
    return false;
  }
}

bool is_auto_gain_enabled(CameraPtr p_cam) {
  try {
    // Check if GainAuto is set to any value other than Off
    return p_cam->GainAuto.GetValue() != GainAuto_Off;
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check if auto gain is enabled: %s", e.what());
    return false;
  }
}

bool is_auto_exposure_enabled(CameraPtr p_cam) {
  try {
    // Check if ExposureAuto is set to any value other than Off
    return p_cam->ExposureAuto.GetValue() != ExposureAuto_Off;
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check if auto exposure is enabled: %s", e.what());
    return false;
  }
}

bool is_auto_white_balance_enabled(CameraPtr p_cam) {
  try {
    // Check if BalanceWhiteAuto is set to any value other than Off
    return p_cam->BalanceWhiteAuto.GetValue() != BalanceWhiteAuto_Off;
  } catch (const Spinnaker::Exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Failed to check if auto white balance is enabled: %s",
                 e.what());
    return false;
  }
}

}  // namespace SpinStereo