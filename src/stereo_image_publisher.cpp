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

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "ImageUtilityStereo.h"
#include "PointCloud.h"
#include "Spinnaker.h"
#include "stereo_image_publisher/spin_stereo_helper.hpp"
#include "stereo_image_publisher/stereo_parameters.hpp"

using namespace Spinnaker;
using namespace std;
using namespace SpinStereo;

class PointCloudGenerator {
 public:
  PointCloudGenerator(CameraPtr p_cam) {};

  bool setPointCloudExtents(INodeMap& node_map,
                            PointCloudParameters& point_cloud_parameters) {
    int64_t stereo_height, stereo_width;
    if (!get_int_value_from_node(node_map, "StereoHeight", stereo_height)) {
      stereo_height = 1536;
    }
    if (!get_int_value_from_node(node_map, "StereoWidth", stereo_width)) {
      stereo_width = 2048;
    }

    point_cloud_parameters.decimationFactor = 4;
    point_cloud_parameters.ROIImageLeft = 0;                      // U Min
    point_cloud_parameters.ROIImageTop = 0;                       // V Min
    point_cloud_parameters.ROIImageRight = (uint)stereo_width;    // U Max
    point_cloud_parameters.ROIImageBottom = (uint)stereo_height;  // V Max

    // Region of Interest (ROI) in World Coordinates
    point_cloud_parameters.ROIWorldCoordinatesXMin = -10.0f;  // X Min
    point_cloud_parameters.ROIWorldCoordinatesXMax = 10.0f;   // X Max
    point_cloud_parameters.ROIWorldCoordinatesYMin = -10.0f;  // Y Min
    point_cloud_parameters.ROIWorldCoordinatesYMax = 10.0f;   // Y Max
    point_cloud_parameters.ROIWorldCoordinatesZMin = 0.0f;    // Z Min
    point_cloud_parameters.ROIWorldCoordinatesZMax = 20.0f;   // Z Max

    return true;
  }

  bool compute_point_cloud(const ImagePtr& disparity_image,
                           const ImagePtr& reference_image, float min_disparity,
                           const StereoCameraParameters& stereo_parameters,
                           const PointCloudParameters& point_cloud_parameters,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) {
    cv::Mat disparity_image_CV_uint16 = cv::Mat(
        disparity_image->GetHeight(), disparity_image->GetWidth(), CV_16UC1,
        disparity_image->GetData(), disparity_image->GetStride());

    unsigned char* reference_image_data =
        static_cast<unsigned char*>(reference_image->GetData());

    point_cloud->points.reserve((disparity_image_CV_uint16.rows /
                                 point_cloud_parameters.decimationFactor) *
                                (disparity_image_CV_uint16.cols /
                                 point_cloud_parameters.decimationFactor));

    for (int i = point_cloud_parameters.ROIImageTop;
         i < std::min(disparity_image_CV_uint16.rows,
                      (int)point_cloud_parameters.ROIImageBottom);
         i += point_cloud_parameters.decimationFactor) {
      for (int j = point_cloud_parameters.ROIImageLeft;
           j < std::min(disparity_image_CV_uint16.cols,
                        (int)point_cloud_parameters.ROIImageRight);
           j += point_cloud_parameters.decimationFactor) {
        ushort disparity_value = disparity_image_CV_uint16.at<ushort>(i, j);

        if (stereo_parameters.invalidDataFlag &&
            disparity_value == stereo_parameters.invalidDataValue) {
          continue;  // Skip invalid disparity
        }

        float disparity =
            disparity_value * stereo_parameters.disparityScaleFactor +
            min_disparity;
        if (disparity <= 0) {
          continue;  // Skip invalid or zero disparity
        }

        // Compute the 3D point (Z = focal_length * baseline / disparity;
        // X = (col - principalPointU) * Z / focal_length;
        // Y = (row - principalPointV) * Z / focal_length)
        float Z = stereo_parameters.focalLength * stereo_parameters.baseline /
                  disparity;
        float X = ((j - stereo_parameters.principalPointU) * Z) /
                  stereo_parameters.focalLength;
        float Y = ((i - stereo_parameters.principalPointV) * Z) /
                  stereo_parameters.focalLength;

        // Skip points outside the world coordinate bounds
        if (X < point_cloud_parameters.ROIWorldCoordinatesXMin ||
            X > point_cloud_parameters.ROIWorldCoordinatesXMax ||
            Y < point_cloud_parameters.ROIWorldCoordinatesYMin ||
            Y > point_cloud_parameters.ROIWorldCoordinatesYMax ||
            Z < point_cloud_parameters.ROIWorldCoordinatesZMin ||
            Z > point_cloud_parameters.ROIWorldCoordinatesZMax) {
          continue;  // Skip points outside of specified world bounds
        }

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;

        // Assign color from reference image
        int pixel_idx = (i * disparity_image_CV_uint16.cols + j) * 3;
        point.r = reference_image_data[pixel_idx];
        point.g = reference_image_data[pixel_idx + 1];
        point.b = reference_image_data[pixel_idx + 2];

        point_cloud->push_back(point);
      }
    }
    return true;
  }

  bool compute_point_cloud(const ImagePtr& disparity_image, float min_disparity,
                           const StereoCameraParameters& stereo_parameters,
                           const PointCloudParameters& point_cloud_parameters,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud) {
    cv::Mat disparity_image_CV_uint16 = cv::Mat(
        disparity_image->GetHeight(), disparity_image->GetWidth(), CV_16UC1,
        disparity_image->GetData(), disparity_image->GetStride());

    point_cloud->points.reserve((disparity_image_CV_uint16.rows /
                                 point_cloud_parameters.decimationFactor) *
                                (disparity_image_CV_uint16.cols /
                                 point_cloud_parameters.decimationFactor));

    for (int i = point_cloud_parameters.ROIImageTop;
         i < std::min(disparity_image_CV_uint16.rows,
                      (int)point_cloud_parameters.ROIImageBottom);
         i += point_cloud_parameters.decimationFactor) {
      for (int j = point_cloud_parameters.ROIImageLeft;
           j < std::min(disparity_image_CV_uint16.cols,
                        (int)point_cloud_parameters.ROIImageRight);
           j += point_cloud_parameters.decimationFactor) {
        ushort disparity_value = disparity_image_CV_uint16.at<ushort>(i, j);

        if (stereo_parameters.invalidDataFlag &&
            disparity_value == stereo_parameters.invalidDataValue) {
          continue;  // Skip invalid disparity
        }

        float disparity =
            disparity_value * stereo_parameters.disparityScaleFactor +
            min_disparity;
        if (disparity <= 0) {
          continue;  // Skip invalid or zero disparity
        }

        // Compute the 3D point (Z = focal_length * baseline / disparity;
        // X = (col - principalPointU) * Z / focal_length;
        // Y = (row - principalPointV) * Z / focal_length)
        float Z = stereo_parameters.focalLength * stereo_parameters.baseline /
                  disparity;
        float X = ((j - stereo_parameters.principalPointU) * Z) /
                  stereo_parameters.focalLength;
        float Y = ((i - stereo_parameters.principalPointV) * Z) /
                  stereo_parameters.focalLength;

        // Skip points outside the world coordinate bounds
        if (X < point_cloud_parameters.ROIWorldCoordinatesXMin ||
            X > point_cloud_parameters.ROIWorldCoordinatesXMax ||
            Y < point_cloud_parameters.ROIWorldCoordinatesYMin ||
            Y > point_cloud_parameters.ROIWorldCoordinatesYMax ||
            Z < point_cloud_parameters.ROIWorldCoordinatesZMin ||
            Z > point_cloud_parameters.ROIWorldCoordinatesZMax) {
          continue;  // Skip points outside of specified world bounds
        }

        pcl::PointXYZRGB point;
        point.x = X;
        point.y = Y;
        point.z = Z;

        point.r =
            (uint8_t)(disparity_value * stereo_parameters.disparityScaleFactor);
        point.g =
            (uint8_t)(disparity_value * stereo_parameters.disparityScaleFactor);
        point.b =
            (uint8_t)(disparity_value * stereo_parameters.disparityScaleFactor);

        point_cloud->push_back(point);
      }
    }
    return true;
  }
};

class StereoImagePublisherNode : public rclcpp::Node {
 public:
  StereoImagePublisherNode(CameraPtr& p_cam,
                           StereoParameters& camera_parameters,
                           PointCloudParameters& point_cloud_parameters,
                           const std::string& node_name)
      : Node(node_name),
        node_name_(node_name),
        p_cam_(p_cam),
        stereo_parameters_(camera_parameters),
        point_cloud_parameters_(point_cloud_parameters) {
    // initialize publisher
    initializePublishers();

    // Declare and initialize parameters
    declareParameters(p_cam_, stereo_parameters_, point_cloud_parameters_);

    // Register a callback for dynamic parameter updates
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StereoImagePublisherNode::parametersCallback, this,
                  std::placeholders::_1));
  }

  void poll_node_map_parameters(CameraPtr p_cam,
                                StereoParameters& camera_parameters,
                                PointCloudParameters& point_cloud_parameters) {
    INodeMap& node_map = p_cam->GetNodeMap();

    // Poll min_disparity (Scan3dCoordinateOffset)
    float scan3d_coordinate_offset;
    if (get_float_value_from_node(node_map, "Scan3dCoordinateOffset",
                                  scan3d_coordinate_offset)) {
      // First if: Update camera_parameters if value changed
      if (std::abs(camera_parameters.scan3d_coordinate_offset -
                   scan3d_coordinate_offset) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters.scan3d_coordinate_offset = scan3d_coordinate_offset;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_min_disp;
    if (this->get_parameter("stereo.min_disparity",
                            current_ros_param_min_disp)) {
      if (std::abs(camera_parameters.scan3d_coordinate_offset -
                   static_cast<float>(current_ros_param_min_disp)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.min_disparity",
            static_cast<double>(camera_parameters.scan3d_coordinate_offset)));
        RCLCPP_DEBUG(this->get_logger(),
                     "min_disparity (Scan3dCoordinateOffset) updated from "
                     "camera node: %f",
                     camera_parameters.scan3d_coordinate_offset);
      }
    }

    // Poll small_penalty
    int64_t small_penalty;
    if (get_int_value_from_node(node_map, "SmallPenalty", small_penalty)) {
      // First if: Update camera_parameters if value changed
      if (camera_parameters.small_penalty != small_penalty) {
        camera_parameters.small_penalty = small_penalty;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_small_penalty;
    if (this->get_parameter("stereo.small_penalty",
                            current_ros_param_small_penalty)) {
      if (camera_parameters.small_penalty != current_ros_param_small_penalty) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.small_penalty",
            static_cast<int>(camera_parameters.small_penalty)));
        RCLCPP_DEBUG(this->get_logger(),
                     "small_penalty updated from camera node: %d",
                     camera_parameters.small_penalty);
      }
    }

    // Poll large_penalty
    int64_t large_penalty;
    if (get_int_value_from_node(node_map, "LargePenalty", large_penalty)) {
      // First if: Update camera_parameters if value changed
      if (camera_parameters.large_penalty != large_penalty) {
        camera_parameters.large_penalty = large_penalty;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_large_penalty;
    if (this->get_parameter("stereo.large_penalty",
                            current_ros_param_large_penalty)) {
      if (camera_parameters.large_penalty != current_ros_param_large_penalty) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.large_penalty",
            static_cast<int>(camera_parameters.large_penalty)));
        RCLCPP_DEBUG(this->get_logger(),
                     "large_penalty updated from camera node: %d",
                     camera_parameters.large_penalty);
      }
    }

    // Poll uniqueness_ratio
    int64_t uniqueness_ratio;
    if (get_int_value_from_node(node_map, "UniquenessRatio",
                                uniqueness_ratio)) {
      // First if: Update camera_parameters if value changed
      if (camera_parameters.uniqueness_ratio != uniqueness_ratio) {
        camera_parameters.uniqueness_ratio = uniqueness_ratio;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_uniqueness_ratio;
    if (this->get_parameter("stereo.uniqueness_ratio",
                            current_ros_param_uniqueness_ratio)) {
      if (camera_parameters.uniqueness_ratio !=
          current_ros_param_uniqueness_ratio) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.uniqueness_ratio",
            static_cast<int>(camera_parameters.uniqueness_ratio)));
        RCLCPP_DEBUG(this->get_logger(),
                     "uniqueness_ratio updated from camera node: %d",
                     camera_parameters.uniqueness_ratio);
      }
    }

    // Poll acquisition frame rate enabled status
    bool acquisition_frame_rate_enabled =
        is_acquisition_frame_rate_enabled(p_cam);
    if (camera_parameters.acquisition_frame_rate_enabled !=
        acquisition_frame_rate_enabled) {
      camera_parameters.acquisition_frame_rate_enabled =
          acquisition_frame_rate_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_acquisition_frame_rate_enabled;
    if (this->get_parameter("camera.acquisition_frame_rate_enabled",
                            current_ros_param_acquisition_frame_rate_enabled)) {
      if (camera_parameters.acquisition_frame_rate_enabled !=
          current_ros_param_acquisition_frame_rate_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "camera.acquisition_frame_rate_enabled",
            camera_parameters.acquisition_frame_rate_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "acquisition_frame_rate_enabled updated from camera node: %s",
            camera_parameters.acquisition_frame_rate_enabled ? "enabled"
                                                             : "disabled");
      }
    }

    // Poll frame_rate
    float frame_rate;
    if (get_frame_rate(p_cam, frame_rate)) {
      if (std::abs(camera_parameters.frame_rate - frame_rate) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters.frame_rate = frame_rate;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_frame_rate;
    if (this->get_parameter("camera.frame_rate",
                            current_ros_param_frame_rate)) {
      if (std::abs(camera_parameters.frame_rate -
                   static_cast<float>(current_ros_param_frame_rate)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.frame_rate",
            static_cast<double>(camera_parameters.frame_rate)));
        RCLCPP_DEBUG(this->get_logger(),
                     "frame_rate updated from camera node: %f",
                     camera_parameters.frame_rate);
      }
    }

    // Poll auto exposure enabled status
    bool auto_exposure = is_auto_exposure_enabled(p_cam);
    if (camera_parameters.auto_exposure != auto_exposure) {
      camera_parameters.auto_exposure = auto_exposure;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_auto_exposure;
    if (this->get_parameter("camera.auto_exposure",
                            current_ros_param_auto_exposure)) {
      if (camera_parameters.auto_exposure != current_ros_param_auto_exposure) {
        this->set_parameter(rclcpp::Parameter("camera.auto_exposure",
                                              camera_parameters.auto_exposure));
        RCLCPP_DEBUG(this->get_logger(),
                     "auto_exposure updated from camera node: %s",
                     camera_parameters.auto_exposure ? "enabled" : "disabled");
      }
    }

    // Poll exposure_time
    float exposure_time;
    if (get_exposure_time(p_cam, exposure_time)) {
      if (std::abs(camera_parameters.exposure_time - exposure_time) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters.exposure_time = exposure_time;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_exposure_time;
    if (this->get_parameter("camera.exposure_time",
                            current_ros_param_exposure_time)) {
      if (std::abs(camera_parameters.exposure_time -
                   static_cast<float>(current_ros_param_exposure_time)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.exposure_time",
            static_cast<double>(camera_parameters.exposure_time)));
        RCLCPP_DEBUG(this->get_logger(),
                     "exposure_time updated from camera node: %f",
                     camera_parameters.exposure_time);
      }
    }

    // Poll auto gain enabled status
    bool auto_gain = is_auto_gain_enabled(p_cam);
    if (camera_parameters.auto_gain != auto_gain) {
      camera_parameters.auto_gain = auto_gain;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_auto_gain;
    if (this->get_parameter("camera.auto_gain", current_ros_param_auto_gain)) {
      if (camera_parameters.auto_gain != current_ros_param_auto_gain) {
        this->set_parameter(
            rclcpp::Parameter("camera.auto_gain", camera_parameters.auto_gain));
        RCLCPP_DEBUG(this->get_logger(),
                     "auto_gain updated from camera node: %s",
                     camera_parameters.auto_gain ? "enabled" : "disabled");
      }
    }

    // Poll gain_value
    float gain_value;
    if (get_gain_value(p_cam, gain_value)) {
      if (std::abs(camera_parameters.gain_value - gain_value) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters.gain_value = gain_value;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_gain_value;
    if (this->get_parameter("camera.gain_value",
                            current_ros_param_gain_value)) {
      if (std::abs(camera_parameters.gain_value -
                   static_cast<float>(current_ros_param_gain_value)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.gain_value",
            static_cast<double>(camera_parameters.gain_value)));
        RCLCPP_DEBUG(this->get_logger(),
                     "gain_value updated from camera node: %f",
                     camera_parameters.gain_value);
      }
    }

    // Poll raw_sensor_1_transmit_enabled
    bool raw_sensor1_transmit_enabled = is_raw_sensor_1_transmit_enabled(p_cam);
    if (camera_parameters.stream_transmit_flags.raw_sensor_1_transmit_enabled !=
        raw_sensor1_transmit_enabled) {
      camera_parameters.stream_transmit_flags.raw_sensor_1_transmit_enabled =
          raw_sensor1_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_raw_sensor1_transmit_enabled;
    if (this->get_parameter("stream.raw_left_enabled",
                            current_ros_param_raw_sensor1_transmit_enabled)) {
      if (camera_parameters.stream_transmit_flags
              .raw_sensor_1_transmit_enabled !=
          current_ros_param_raw_sensor1_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.raw_left_enabled", camera_parameters.stream_transmit_flags
                                           .raw_sensor_1_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "raw_sensor_1_transmit_enabled updated from camera node: %s",
            camera_parameters.stream_transmit_flags
                    .raw_sensor_1_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll raw_sensor_2_transmit_enabled
    bool raw_sensor2_transmit_enabled = is_raw_sensor_2_transmit_enabled(p_cam);
    if (camera_parameters.stream_transmit_flags.raw_sensor_2_transmit_enabled !=
        raw_sensor2_transmit_enabled) {
      camera_parameters.stream_transmit_flags.raw_sensor_2_transmit_enabled =
          raw_sensor2_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_raw_sensor2_transmit_enabled;
    if (this->get_parameter("stream.raw_right_enabled",
                            current_ros_param_raw_sensor2_transmit_enabled)) {
      if (camera_parameters.stream_transmit_flags
              .raw_sensor_2_transmit_enabled !=
          current_ros_param_raw_sensor2_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.raw_right_enabled", camera_parameters.stream_transmit_flags
                                            .raw_sensor_2_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "raw_sensor_2_transmit_enabled updated from camera node: %s",
            camera_parameters.stream_transmit_flags
                    .raw_sensor_2_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll rect_sensor_1_transmit_enabled
    bool rect_sensor1_transmit_enabled =
        is_rectified_sensor_1_transmit_enabled(p_cam);
    if (camera_parameters.stream_transmit_flags
            .rect_sensor_1_transmit_enabled != rect_sensor1_transmit_enabled) {
      camera_parameters.stream_transmit_flags.rect_sensor_1_transmit_enabled =
          rect_sensor1_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_rect_sensor1_transmit_enabled;
    if (this->get_parameter("stream.rect_left_enabled",
                            current_ros_param_rect_sensor1_transmit_enabled)) {
      if (camera_parameters.stream_transmit_flags
              .rect_sensor_1_transmit_enabled !=
          current_ros_param_rect_sensor1_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.rect_left_enabled", camera_parameters.stream_transmit_flags
                                            .rect_sensor_1_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "rect_sensor_1_transmit_enabled updated from camera node: %s",
            camera_parameters.stream_transmit_flags
                    .rect_sensor_1_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll rect_sensor_2_transmit_enabled
    bool rect_sensor2_transmit_enabled =
        is_rectified_sensor_2_transmit_enabled(p_cam);
    if (camera_parameters.stream_transmit_flags
            .rect_sensor_2_transmit_enabled != rect_sensor2_transmit_enabled) {
      camera_parameters.stream_transmit_flags.rect_sensor_2_transmit_enabled =
          rect_sensor2_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_rect_sensor2_transmit_enabled;
    if (this->get_parameter("stream.rect_right_enabled",
                            current_ros_param_rect_sensor2_transmit_enabled)) {
      if (camera_parameters.stream_transmit_flags
              .rect_sensor_2_transmit_enabled !=
          current_ros_param_rect_sensor2_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.rect_right_enabled", camera_parameters.stream_transmit_flags
                                             .rect_sensor_2_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "rect_sensor_2_transmit_enabled updated from camera node: %s",
            camera_parameters.stream_transmit_flags
                    .rect_sensor_2_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll disparity_transmit_enabled
    bool disparity_transmit_enabled = is_disparity_transmit_enabled(p_cam);
    if (camera_parameters.stream_transmit_flags.disparity_transmit_enabled !=
        disparity_transmit_enabled) {
      camera_parameters.stream_transmit_flags.disparity_transmit_enabled =
          disparity_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_disparity_transmit_enabled;
    if (this->get_parameter("stream.disparity_enabled",
                            current_ros_param_disparity_transmit_enabled)) {
      if (camera_parameters.stream_transmit_flags.disparity_transmit_enabled !=
          current_ros_param_disparity_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.disparity_enabled", camera_parameters.stream_transmit_flags
                                            .disparity_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "disparity_transmit_enabled updated from camera node: %s",
            camera_parameters.stream_transmit_flags.disparity_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }
  }

  void initializePublishers() {
    // Reset all existing publishers before reinitializing
    raw_left_image_publisher_.reset();
    raw_right_image_publisher_.reset();
    rect_left_image_publisher_.reset();
    rect_right_image_publisher_.reset();
    disparity_image_publisher_.reset();
    point_cloud_publisher_.reset();

    // Get the camera's serial number
    std::string serial_number =
        "serial_" +
        std::string(p_cam_->TLDevice.DeviceSerialNumber.GetValue().c_str());

    // Conditionally create publishers based on the transmission flags
    if (stereo_parameters_.stream_transmit_flags
            .raw_sensor_1_transmit_enabled) {
      raw_left_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/raw_left_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Raw left image publisher created.");
    }

    if (stereo_parameters_.stream_transmit_flags
            .raw_sensor_2_transmit_enabled) {
      raw_right_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/raw_right_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Raw right image publisher created.");
    }

    if (stereo_parameters_.stream_transmit_flags
            .rect_sensor_1_transmit_enabled) {
      rect_left_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/rectified_left_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(),
                  "Rectified left image publisher created.");
    }

    if (stereo_parameters_.stream_transmit_flags
            .rect_sensor_2_transmit_enabled) {
      rect_right_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/rectified_right_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(),
                  "Rectified right image publisher created.");
    }

    if (stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled) {
      disparity_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/disparity_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Disparity image publisher created.");
    }

    if (stereo_parameters_.do_compute_point_cloud &&
        stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled) {
      point_cloud_publisher_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>(
              "Bumblebee_X/" + serial_number + "/point_cloud",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Point cloud publisher created.");
    }
  }

  void publishImage(
      ImagePtr imagePtr,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
      const std::string& encoding, rclcpp::Time time_stamp) {
    if (publisher) {
      // Check if image data is valid
      if (imagePtr->GetData() == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("publishImage"),
                     "Error: Image data is null");
        return;
      }
      try {
        cv::Mat cvImage =
            cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(), CV_8UC3,
                    (unsigned char*)imagePtr->GetData(), imagePtr->GetStride());

        // Convert to ROS Image message
        auto msg =
            cv_bridge::CvImage(std_msgs::msg::Header(), encoding, cvImage)
                .toImageMsg();
        msg->header.stamp = time_stamp;
        publisher->publish(*msg);

      } catch (const cv::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("publishImage"), "OpenCV error: %s",
                     e.what());
        return;
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("publishImage"),
                   "Publisher is not initialized");
    }
  }

  void publishDisparityImage(ImagePtr imagePtr, rclcpp::Time time_stamp) {
    if (disparity_image_publisher_) {
      cv::Mat disparity_image =
          cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(), CV_16UC1,
                  (unsigned char*)imagePtr->GetData(), imagePtr->GetStride());

      // defaulting to always use colormap for better visualization.
      // To see how to use the 16 bit disparity image, see the else{} block.
      bool applyColorMap = true;

      // Convert to ROS Image message
      if (applyColorMap) {
        cv::Mat disparity_image(imagePtr->GetHeight(), imagePtr->GetWidth(),
                                CV_16UC1, (unsigned char*)imagePtr->GetData(),
                                imagePtr->GetStride());

        // Convert disparity image from 16-bit to 8-bit and scale to [0, 255]
        cv::Mat disparity8bit;
        disparity_image.convertTo(disparity8bit, CV_8UC1, 255.0 / 16384.0);

        // Apply a colormap for better visualization
        cv::Mat coloredDisparity;
        cv::applyColorMap(disparity8bit, coloredDisparity, cv::COLORMAP_JET);

        // Create a mask for invalid disparity values (where disparity is zero)
        cv::Mat mask = (disparity_image == 0);
        mask.convertTo(mask, CV_8UC1);  // Ensure mask is 8-bit

        // Set invalid pixels to black in the colored disparity image
        coloredDisparity.setTo(cv::Scalar(0, 0, 0), mask);

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8",
                                      coloredDisparity)
                       .toImageMsg();
        msg->header.stamp = time_stamp;
        disparity_image_publisher_->publish(*msg);
      }
      // If computing point cloud or getting a depth image, the momo16 (16UC1)
      // format should be published.
      else {
        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "16UC1",
                                      disparity_image)
                       .toImageMsg();
        msg->header.stamp = time_stamp;
        disparity_image_publisher_->publish(*msg);
      }
    }
  }

  void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud,
                         rclcpp::Time time_stamp) {
    if (point_cloud_publisher_) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*point_cloud, msg);
      msg.header.frame_id = "camera_link";
      msg.header.stamp = time_stamp;
      point_cloud_publisher_->publish(msg);
    }
  }

  // Getter methods to access the private publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getRawLeftPublisher() {
    return raw_left_image_publisher_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getRawRightPublisher() {
    return raw_right_image_publisher_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getRectLeftPublisher() {
    return rect_left_image_publisher_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
  getRectRightPublisher() {
    return rect_right_image_publisher_;
  }

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
  getDisparityPublisher() {
    return disparity_image_publisher_;
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
  getPointCloudPublisher() {
    return point_cloud_publisher_;
  }

 private:
  std::string node_name_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      raw_left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      raw_right_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      rect_left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      rect_right_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      disparity_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  CameraPtr& p_cam_;
  StereoParameters& stereo_parameters_;
  PointCloudParameters& point_cloud_parameters_;

  void declareParameters(CameraPtr p_cam_, StereoParameters stereo_parameters_,
                         PointCloudParameters point_cloud_parameters_) {
    INodeMap& nodeMap_ = p_cam_->GetNodeMap();
    // stereo parameters
    // Scan3dCoordinateOffset (min_disparity) parameter
    rcl_interfaces::msg::ParameterDescriptor minDisp_desc;
    float minScan3dCoordinateOffset, maxScan3dCoordinateOffset;
    get_min_float_value_from_node(nodeMap_, "Scan3dCoordinateOffset",
                                  minScan3dCoordinateOffset);
    get_max_float_value_from_node(nodeMap_, "Scan3dCoordinateOffset",
                                  maxScan3dCoordinateOffset);
    minDisp_desc.description =
        "Minimum Scan3d coordinate offset (min disparity), clamped between " +
        std::to_string(minScan3dCoordinateOffset) + " and " +
        std::to_string(maxScan3dCoordinateOffset);
    minDisp_desc.floating_point_range.push_back(
        rcl_interfaces::msg::FloatingPointRange());
    minDisp_desc.floating_point_range[0].from_value = minScan3dCoordinateOffset;
    minDisp_desc.floating_point_range[0].to_value = maxScan3dCoordinateOffset;
    this->declare_parameter<double>("stereo.min_disparity",
                                    stereo_parameters_.scan3d_coordinate_offset,
                                    minDisp_desc);

    // small_penalty parameter with clamping between 0 and 240
    rcl_interfaces::msg::ParameterDescriptor smallPenalty_desc;
    int64_t minSmallPenalty, maxSmallPenalty;
    get_min_int_value_from_node(nodeMap_, "SmallPenalty", minSmallPenalty);
    get_max_int_value_from_node(nodeMap_, "SmallPenalty", maxSmallPenalty);
    smallPenalty_desc.description = "Small penalty, clamped between " +
                                    std::to_string(minSmallPenalty) + " and " +
                                    std::to_string(maxSmallPenalty);
    smallPenalty_desc.integer_range.push_back(
        rcl_interfaces::msg::IntegerRange());
    smallPenalty_desc.integer_range[0].from_value = minSmallPenalty;
    smallPenalty_desc.integer_range[0].to_value = maxSmallPenalty;
    this->declare_parameter<int>("stereo.small_penalty",
                                 stereo_parameters_.small_penalty,
                                 smallPenalty_desc);

    // large_penalty parameter with clamping between 1 and 255
    rcl_interfaces::msg::ParameterDescriptor largePenalty_desc;
    int64_t minLargePenalty, maxLargePenalty;
    get_min_int_value_from_node(nodeMap_, "LargePenalty", minLargePenalty);
    get_max_int_value_from_node(nodeMap_, "LargePenalty", maxLargePenalty);
    largePenalty_desc.description = "Large penalty, clamped between " +
                                    std::to_string(minLargePenalty) + " and " +
                                    std::to_string(maxLargePenalty);
    largePenalty_desc.integer_range.push_back(
        rcl_interfaces::msg::IntegerRange());
    largePenalty_desc.integer_range[0].from_value = minLargePenalty;
    largePenalty_desc.integer_range[0].to_value = maxLargePenalty;
    this->declare_parameter<int>("stereo.large_penalty",
                                 stereo_parameters_.large_penalty,
                                 largePenalty_desc);

    // large_penalty parameter with clamping between 1 and 255
    int64_t minUniquenessRatio, maxUniquenessRatio;
    get_min_int_value_from_node(nodeMap_, "UniquenessRatio",
                                minUniquenessRatio);
    get_max_int_value_from_node(nodeMap_, "UniquenessRatio",
                                maxUniquenessRatio);
    rcl_interfaces::msg::ParameterDescriptor uniquenessRatio_desc;
    uniquenessRatio_desc.description = "Uniqueness ratio, clamped between " +
                                       std::to_string(minUniquenessRatio) +
                                       " and " +
                                       std::to_string(maxUniquenessRatio);
    uniquenessRatio_desc.integer_range.push_back(
        rcl_interfaces::msg::IntegerRange());
    uniquenessRatio_desc.integer_range[0].from_value = minUniquenessRatio;
    uniquenessRatio_desc.integer_range[0].to_value = maxUniquenessRatio;
    this->declare_parameter<int>("stereo.uniqueness_ratio",
                                 stereo_parameters_.uniqueness_ratio,
                                 uniquenessRatio_desc);

    // post processing parameters
    this->declare_parameter<bool>("post_processing.enable",
                                  stereo_parameters_.post_process_disparity);
    this->declare_parameter<int>("post_processing.max_speckle_size",
                                 stereo_parameters_.max_speckle_size);
    this->declare_parameter<double>("post_processing.max_diff",
                                    stereo_parameters_.max_diff);

    // stream transmit parameters
    this->declare_parameter<bool>(
        "stream.raw_left_enabled",
        stereo_parameters_.stream_transmit_flags.raw_sensor_1_transmit_enabled);
    this->declare_parameter<bool>(
        "stream.raw_right_enabled",
        stereo_parameters_.stream_transmit_flags.raw_sensor_2_transmit_enabled);
    this->declare_parameter<bool>("stream.rect_left_enabled",
                                  stereo_parameters_.stream_transmit_flags
                                      .rect_sensor_1_transmit_enabled);
    this->declare_parameter<bool>("stream.rect_right_enabled",
                                  stereo_parameters_.stream_transmit_flags
                                      .rect_sensor_2_transmit_enabled);
    this->declare_parameter<bool>(
        "stream.disparity_enabled",
        stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled);

    // camera parameters
    this->declare_parameter<bool>(
        "camera.acquisition_frame_rate_enabled",
        stereo_parameters_.acquisition_frame_rate_enabled);
    this->declare_parameter<double>("camera.frame_rate",
                                    stereo_parameters_.frame_rate);
    this->declare_parameter<bool>("camera.auto_exposure",
                                  stereo_parameters_.auto_exposure);
    this->declare_parameter<double>("camera.exposure_time",
                                    stereo_parameters_.exposure_time);
    this->declare_parameter<bool>("camera.auto_gain",
                                  stereo_parameters_.auto_gain);
    this->declare_parameter<double>("camera.gain_value",
                                    stereo_parameters_.gain_value);

    // point cloud parameters
    this->declare_parameter<bool>("point_cloud.enable",
                                  stereo_parameters_.do_compute_point_cloud);
    this->declare_parameter<int>("point_cloud.decimation_factor",
                                 point_cloud_parameters_.decimationFactor);
    this->declare_parameter<float>(
        "point_cloud.XMin",
        point_cloud_parameters_.ROIWorldCoordinatesXMin);  // X Min
    this->declare_parameter<float>(
        "point_cloud.XMax",
        point_cloud_parameters_.ROIWorldCoordinatesXMax);  // X Max
    this->declare_parameter<float>(
        "point_cloud.YMin",
        point_cloud_parameters_.ROIWorldCoordinatesYMin);  // Y Min
    this->declare_parameter<float>(
        "point_cloud.YMax",
        point_cloud_parameters_.ROIWorldCoordinatesYMax);  // Y Max
    this->declare_parameter<float>(
        "point_cloud.ZMin",
        point_cloud_parameters_.ROIWorldCoordinatesZMin);  // Z Min
    this->declare_parameter<float>(
        "point_cloud.ZMax",
        point_cloud_parameters_.ROIWorldCoordinatesZMax);  // Z Max

    // Region of Interest (ROI) in Image Coordinates
    this->declare_parameter<int>(
        "point_cloud.ROI_image_left",
        point_cloud_parameters_.ROIImageLeft);  // U Min
    this->declare_parameter<int>("point_cloud.ROI_image_top",
                                 point_cloud_parameters_.ROIImageTop);  // V Min
    this->declare_parameter<int>(
        "point_cloud.ROI_image_right",
        point_cloud_parameters_.ROIImageRight);  // U Max
    this->declare_parameter<int>(
        "point_cloud.ROI_image_bottom",
        point_cloud_parameters_.ROIImageBottom);  // V Max
  }

  // Callback to handle parameter updates
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& params) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool updatePublisher = false;

    for (const auto& param : params) {
      if (param.get_name() == "stereo.min_disparity") {
        double new_value_double = param.as_double();
        float new_value = static_cast<float>(new_value_double);
        if (!set_scan3d_coordinate_offset(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the Scan3dCoordinateOffset node.");
          result.successful = false;
        }
        stereo_parameters_.scan3d_coordinate_offset = new_value;
        RCLCPP_INFO(this->get_logger(), "min_disparity updated: %f",
                    stereo_parameters_.scan3d_coordinate_offset);
      } else if (param.get_name() == "stereo.small_penalty") {
        int new_value = param.as_int();
        if (new_value >= stereo_parameters_.large_penalty) {
          RCLCPP_WARN(
              this->get_logger(),
              "small_penalty must be less than large_penalty. Clamping value.");
          new_value = stereo_parameters_.large_penalty - 1;
        }
        if (!set_small_penalty(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the SmallPenalty node.");
          result.successful = false;
        }
        stereo_parameters_.small_penalty = new_value;
        RCLCPP_INFO(this->get_logger(), "small_penalty updated: %d",
                    stereo_parameters_.small_penalty);
      } else if (param.get_name() == "stereo.large_penalty") {
        int new_value = param.as_int();
        if (new_value <= stereo_parameters_.small_penalty) {
          RCLCPP_WARN(this->get_logger(),
                      "large_penalty must be greater than small_penalty. "
                      "Clamping value.");
          new_value = stereo_parameters_.small_penalty + 1;
        }
        if (!set_large_penalty(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the LargePenalty node.");
          result.successful = false;
        }
        stereo_parameters_.large_penalty = new_value;
        RCLCPP_INFO(this->get_logger(), "large_penalty updated: %d",
                    stereo_parameters_.large_penalty);
      } else if (param.get_name() == "stereo.uniqueness_ratio") {
        int new_value = param.as_int();
        if (!set_uniqueness_ratio(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the UniquenessRatio node.");
          result.successful = false;
        }
        stereo_parameters_.uniqueness_ratio = new_value;
        RCLCPP_INFO(this->get_logger(), "uniqueness_ratio updated: %d",
                    stereo_parameters_.uniqueness_ratio);
      } else if (param.get_name() == "post_processing.enable") {
        stereo_parameters_.post_process_disparity = param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "Post Processing Disparity: %s",
            stereo_parameters_.post_process_disparity ? "enabled" : "disabled");
      } else if (param.get_name() == "post_processing.max_speckle_size") {
        stereo_parameters_.max_speckle_size = param.as_int();
        RCLCPP_INFO(this->get_logger(), "max_speckle_size updated: %d",
                    stereo_parameters_.max_speckle_size);
      } else if (param.get_name() == "post_processing.max_diff") {
        stereo_parameters_.max_diff = param.as_double();
        RCLCPP_INFO(this->get_logger(), "max_diff updated: %f",
                    stereo_parameters_.max_diff);
      } else if (param.get_name() == "stream.raw_left_enabled") {
        stereo_parameters_.stream_transmit_flags.raw_sensor_1_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rawLeftEnabled updated: %s",
                    stereo_parameters_.stream_transmit_flags
                            .raw_sensor_1_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (param.get_name() == "stream.raw_right_enabled") {
        stereo_parameters_.stream_transmit_flags.raw_sensor_2_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rawRightEnabled updated: %s",
                    stereo_parameters_.stream_transmit_flags
                            .raw_sensor_2_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (param.get_name() == "stream.rect_left_enabled") {
        stereo_parameters_.stream_transmit_flags
            .rect_sensor_1_transmit_enabled = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rectLeftEnabled updated: %s",
                    stereo_parameters_.stream_transmit_flags
                            .rect_sensor_1_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (param.get_name() == "stream.rect_right_enabled") {
        stereo_parameters_.stream_transmit_flags
            .rect_sensor_2_transmit_enabled = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rectRightEnabled updated: %s",
                    stereo_parameters_.stream_transmit_flags
                            .rect_sensor_2_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (param.get_name() == "stream.disparity_enabled") {
        stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "disparity_transmit_enabled updated: %s",
            stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled
                ? "enabled"
                : "disabled");
        updatePublisher = true;
        if (!stereo_parameters_.stream_transmit_flags
                 .disparity_transmit_enabled) {
          if (stereo_parameters_.do_compute_point_cloud) {
            RCLCPP_WARN(rclcpp::get_logger(node_name_),
                        "Need the disparity transmission enabled to compute "
                        "point cloud.");
          }
        }
      } else if (param.get_name() == "camera.acquisition_frame_rate_enabled") {
        bool new_value = param.as_bool();
        if (new_value != stereo_parameters_.acquisition_frame_rate_enabled) {
          stereo_parameters_.acquisition_frame_rate_enabled = new_value;
          if (!set_acquisition_frame_rate_enabled(
                  p_cam_, stereo_parameters_.acquisition_frame_rate_enabled)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set the AcquisitionFrameRateEnable node.");
            result.successful = false;
          }
          RCLCPP_INFO(
              this->get_logger(), "acquisition_frame_rate_enabled updated: %s",
              stereo_parameters_.acquisition_frame_rate_enabled ? "enabled"
                                                                : "disabled");
        }
      } else if (param.get_name() == "camera.frame_rate") {
        double new_frame_rate_double = param.as_double();
        float new_frame_rate = static_cast<float>(new_frame_rate_double);
        if (std::abs(new_frame_rate - stereo_parameters_.frame_rate) > 1e-6) {
          // stereo_parameters_.acquisition_frame_rate_enabled = true;
          if (!set_frame_rate(p_cam_, new_frame_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the frame rate.");
            result.successful = false;
          }
          if (get_frame_rate(p_cam_, stereo_parameters_.frame_rate)) {
            RCLCPP_INFO(this->get_logger(), "frame_rate updated: %f",
                        stereo_parameters_.frame_rate);
            stereo_parameters_.frame_rate = stereo_parameters_.frame_rate;
          }
        }
      } else if (param.get_name() == "camera.auto_exposure") {
        bool new_value = param.as_bool();
        if (new_value != stereo_parameters_.auto_exposure) {
          stereo_parameters_.auto_exposure = new_value;
          if (new_value == true) {
            if (!enable_auto_exposure(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to set the ExposureAuto to continuous.");
            }
          } else {
            if (!disable_auto_exposure(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to set the ExposureAuto to Off.");
            }
          }
          RCLCPP_INFO(
              this->get_logger(), "auto_exposure updated: %s",
              stereo_parameters_.auto_exposure ? "enabled" : "disabled");
        }
      } else if (param.get_name() == "camera.exposure_time") {
        double new_exposure_time_double = param.as_double();
        float new_exposure_time = static_cast<float>(new_exposure_time_double);
        if (std::abs(new_exposure_time - stereo_parameters_.exposure_time) >
            1e-6) {
          // stereo_parameters_.auto_exposure = false;
          stereo_parameters_.exposure_time = new_exposure_time;
          if (!set_exposure_time(p_cam_, new_exposure_time)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set the exposure time.");
            result.successful = false;
          }
          RCLCPP_INFO(this->get_logger(), "exposure_time updated: %f",
                      stereo_parameters_.exposure_time);
        }
      } else if (param.get_name() == "camera.auto_gain") {
        bool new_value = param.as_bool();
        if (new_value != stereo_parameters_.auto_gain) {
          stereo_parameters_.auto_gain = new_value;
          if (new_value == true) {
            if (!enable_auto_gain(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to set the auto_gain to continuous.");
            }
          } else {
            if (!disable_auto_gain(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to set the auto_gain to Off.");
            }
          }
          RCLCPP_INFO(this->get_logger(), "auto_gain updated: %s",
                      stereo_parameters_.auto_gain ? "enabled" : "disabled");
        }
      } else if (param.get_name() == "camera.gain_value") {
        double new_gain_value_double = param.as_double();
        float new_gain_value = static_cast<float>(new_gain_value_double);
        if (std::abs(new_gain_value - stereo_parameters_.gain_value) > 1e-6) {
          // stereo_parameters_.auto_gain = false;
          stereo_parameters_.gain_value = new_gain_value;
          if (!set_gain_value(p_cam_, new_gain_value)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set the exposure time.");
            result.successful = false;
          }
          RCLCPP_INFO(this->get_logger(), "exposure_time updated: %f",
                      stereo_parameters_.gain_value);
        }
      } else if (param.get_name() == "point_cloud.enable") {
        stereo_parameters_.do_compute_point_cloud = param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "Point cloud transmission: %s",
            stereo_parameters_.do_compute_point_cloud ? "enabled" : "disabled");
        updatePublisher = true;
        if (stereo_parameters_.do_compute_point_cloud) {
          if (!stereo_parameters_.stream_transmit_flags
                   .disparity_transmit_enabled) {
            RCLCPP_WARN(rclcpp::get_logger(node_name_),
                        "Need the disparity transmission enabled to compute "
                        "point cloud.");
          }
        }
      } else if (param.get_name() == "point_cloud.decimation_factor") {
        int new_value = param.as_int();
        if (new_value < 1) {
          RCLCPP_WARN(
              this->get_logger(),
              "decimationFactor must be greater than 1. Clamping value.");
          new_value = 1;
        }
        point_cloud_parameters_.decimationFactor = new_value;
        RCLCPP_INFO(this->get_logger(), "decimationFactor updated: %d",
                    point_cloud_parameters_.decimationFactor);
      } else if (param.get_name() == "point_cloud.XMin") {
        point_cloud_parameters_.ROIWorldCoordinatesXMin = param.as_double();
        RCLCPP_INFO(this->get_logger(), "XMin updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesXMin);
      } else if (param.get_name() == "point_cloud.XMax") {
        point_cloud_parameters_.ROIWorldCoordinatesXMax = param.as_double();
        RCLCPP_INFO(this->get_logger(), "XMax updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesXMax);
      } else if (param.get_name() == "point_cloud.YMin") {
        point_cloud_parameters_.ROIWorldCoordinatesYMin = param.as_double();
        RCLCPP_INFO(this->get_logger(), "YMin updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesYMin);
      } else if (param.get_name() == "point_cloud.YMax") {
        point_cloud_parameters_.ROIWorldCoordinatesYMax = param.as_double();
        RCLCPP_INFO(this->get_logger(), "YMax updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesYMax);
      } else if (param.get_name() == "point_cloud.ZMin") {
        point_cloud_parameters_.ROIWorldCoordinatesZMin = param.as_double();
        RCLCPP_INFO(this->get_logger(), "ZMin updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesZMin);
      } else if (param.get_name() == "point_cloud.ZMax") {
        point_cloud_parameters_.ROIWorldCoordinatesZMax = param.as_double();
        RCLCPP_INFO(this->get_logger(), "ZMax updated: %f",
                    point_cloud_parameters_.ROIWorldCoordinatesZMax);
      } else if (param.get_name() == "point_cloud.ROI_image_left") {
        point_cloud_parameters_.ROIImageLeft = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROIImageLeft updated: %d",
                    point_cloud_parameters_.ROIImageLeft);
      } else if (param.get_name() == "point_cloud.ROI_image_top") {
        point_cloud_parameters_.ROIImageTop = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROIImageTop updated: %d",
                    point_cloud_parameters_.ROIImageTop);
      } else if (param.get_name() == "point_cloud.ROI_image_right") {
        point_cloud_parameters_.ROIImageRight = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROIImageRight updated: %d",
                    point_cloud_parameters_.ROIImageRight);
      } else if (param.get_name() == "point_cloud.ROI_image_bottom") {
        point_cloud_parameters_.ROIImageBottom = param.as_int();
        RCLCPP_INFO(this->get_logger(), "ROIImageBottom updated: %d",
                    point_cloud_parameters_.ROIImageBottom);
      }
    }

    if (updatePublisher) {
      RCLCPP_INFO(this->get_logger(), "Configuring streams");
      try {
        if (p_cam_->IsStreaming()) {
          p_cam_->EndAcquisition();
        }

        // if all the streams are disabled, keep the acquisition off
        bool at_least_one_enabled =
            stereo_parameters_.stream_transmit_flags
                .raw_sensor_1_transmit_enabled ||
            stereo_parameters_.stream_transmit_flags
                .raw_sensor_2_transmit_enabled ||
            stereo_parameters_.stream_transmit_flags
                .rect_sensor_1_transmit_enabled ||
            stereo_parameters_.stream_transmit_flags
                .rect_sensor_2_transmit_enabled ||
            stereo_parameters_.stream_transmit_flags.disparity_transmit_enabled;

        if (at_least_one_enabled) {
          // Set throughput to max
          RCLCPP_DEBUG(
              rclcpp::get_logger(node_name_),
              "Trying to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
          if (!set_device_link_throughput_to_max(p_cam_)) {
            RCLCPP_WARN(rclcpp::get_logger(node_name_),
                        "Unable to set DeviceLinkThroughputLimit to "
                        "DeviceMaxThroughput");
          }

          // Reconfigure the camera streams
          if (!SpinStereo::configure_camera_streams(
                  p_cam_, stereo_parameters_.stream_transmit_flags)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error during stream reconfiguration: %s",
                         "Could not configure camera streams.");
            result.successful = false;
            return result;
          }

          // Set throughput to current
          RCLCPP_DEBUG(rclcpp::get_logger(node_name_),
                       "Trying to set DeviceLinkThroughputLimit to "
                       "DeviceLinkCurrentThroughput");
          if (!set_device_link_throughput_to_current(p_cam_)) {
            RCLCPP_WARN(rclcpp::get_logger(node_name_),
                        "Unable to set DeviceLinkThroughputLimit to "
                        "DeviceLinkCurrentThroughput. "
                        "This might cause packet loss.");
          }

          // if more streams are enabled, the frame rate might change. Updating
          // the resulting frame rate accordingly.
          if (!get_frame_rate(p_cam_, stereo_parameters_.frame_rate)) {
            RCLCPP_DEBUG(this->get_logger(),
                         "Could not read the updated frame rate. This might "
                         "cause issues in timeout.");
          }

          if (!p_cam_->IsStreaming()) {
            p_cam_->BeginAcquisition();
          }
        } else {
          RCLCPP_WARN(this->get_logger(), "All streams have been disabled.");
        }

        initializePublishers();

      } catch (const Spinnaker::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Error during stream reconfiguration: %s", e.what());
        result.successful = false;
      }
    }
    return result;
  }
};

int main(int argc, char** argv) {
  // Initialize ROS2
  rclcpp::init(argc, argv);

  // Create an instance of the stereo parameters object
  StereoParameters camera_parameters;

  // Create an instance of the stereo parameters object
  StereoCameraParameters stereo_parameters;

  // Retrieve a singleton reference to the Spinnaker system object
  SystemPtr system = System::GetInstance();

  // Retrieve the list of available cameras from the system
  CameraList camList = system->GetCameras();

  // Get the number of connected cameras
  const unsigned int numCameras = camList.GetSize();

  // If no cameras are found, log an error and release resources
  if (numCameras == 0) {
    camList.Clear();            // Clear the camera list to release resources
    system->ReleaseInstance();  // Release the Spinnaker system instance
    RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                 "Not enough cameras!");
    return -1;  // Return error code as no cameras are available
  }

  // create a camera pointer
  CameraPtr p_cam = nullptr;

  // the serial number for the camera
  std::string serial_number = "";

  // if the serial_number is given
  if (argc > 1) {
    // get the first argument (serial number)
    serial_number = argv[1];

    // Iterate through cameras to find one with the matching serial number
    for (unsigned int i = 0; i < numCameras; ++i) {
      CameraPtr cam = camList.GetByIndex(i);
      std::string camera_serial =
          cam->TLDevice.DeviceSerialNumber.GetValue().c_str();
      if (serial_number == camera_serial) {
        // Check to make sure camera supports stereo vision
        RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                     "Checking camera stereo support...");
        if (!ImageUtilityStereo::IsStereoCamera(cam)) {
          RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                       "Camera with serial number %s is not a valid BumbleBee "
                       "X camera. Abborting...",
                       cam->TLDevice.DeviceSerialNumber.GetValue().c_str());
          camList.Clear();  // Clear the camera list to release resources
          system->ReleaseInstance();  // Release the Spinnaker system instance
          return -1;  // Return error code as no cameras are available
        }
        p_cam = cam;
        break;
      }
    }

    // If no matching camera was found, release resources and return an error
    if (!p_cam) {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "Camera with serial number %s not found!",
                   serial_number.c_str());
      camList.Clear();
      system->ReleaseInstance();
      return -1;
    }
  } else {
    // If the camera serial number is not defined, cycle through cameras to get
    // the first BumbleBee X.
    for (unsigned int i = 0; i < numCameras; ++i) {
      CameraPtr cam = camList.GetByIndex(i);
      if (!ImageUtilityStereo::IsStereoCamera(cam)) {
        RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                     "Camera with serial number %s is not a valid BumbleBee X "
                     "camera. Checking other cameras...",
                     cam->TLDevice.DeviceSerialNumber.GetValue().c_str());
        continue;
      }
      p_cam = cam;
      serial_number = p_cam->TLDevice.DeviceSerialNumber.GetValue().c_str();
      break;
    }

    // If no matching camera was found, release resources and return an error
    if (!p_cam) {
      RCLCPP_ERROR(rclcpp::get_logger("stereo_image_publisher"),
                   "No BumbleBee X detected. Aborting.");
      camList.Clear();
      system->ReleaseInstance();
      return -1;
    }
  }
  // Initialize the camera for usage
  p_cam->Init();

  // Retrieve the GenICam node map (which contains the camera settings) for
  // configuration
  INodeMap& node_map_camera = p_cam->GetNodeMap();

  // get the node name with the serial number
  std::string node_name = "stereo_image_publisher_" + serial_number;

  // Acquire and publish images
  RCLCPP_INFO(rclcpp::get_logger(node_name), "Configuring camera streams...");
  if (!configure_acquisition(p_cam, camera_parameters.stream_transmit_flags)) {
    RCLCPP_ERROR(rclcpp::get_logger(node_name),
                 "Unable to configure camera streams. Aborting...");
    return -1;
  }

  RCLCPP_DEBUG(rclcpp::get_logger(node_name),
               "Configuring device link throughput...");
  if (!set_device_link_throughput(p_cam)) {
    RCLCPP_WARN(rclcpp::get_logger(node_name),
                "Unable to set device link throughput.");
  }

  RCLCPP_DEBUG(rclcpp::get_logger(node_name),
               "Configuring stereo processing...");
  if (!configure_stereo_processing(node_map_camera, stereo_parameters)) {
    RCLCPP_WARN(rclcpp::get_logger(node_name),
                "Unable to configure stereo camera parameters.");
  }

  RCLCPP_DEBUG(rclcpp::get_logger(node_name),
               "Setting acquisition frame rate...");
  if (!set_frame_rate(p_cam, camera_parameters.frame_rate)) {
    RCLCPP_WARN(rclcpp::get_logger(node_name),
                "Unable to set acquisition frame rate.");
  } else {
    if (!get_frame_rate(p_cam, camera_parameters.frame_rate)) {
      RCLCPP_WARN(rclcpp::get_logger(node_name),
                  "Unable to get acquisition frame rate.");
    } else {
      RCLCPP_DEBUG(rclcpp::get_logger(node_name), "frame_rate updated: %f",
                   camera_parameters.frame_rate);
    }
  }

  if (!p_cam->IsStreaming()) {
    RCLCPP_INFO(rclcpp::get_logger(node_name), "Begin acquisition...");
    p_cam->BeginAcquisition();
  }

  // Initialize the parameters for the point cloud generation.
  PointCloudParameters point_cloud_parameters;

  // Create an instance of the PointCloudGenerator class, passing the camera
  // pointer (p_cam) to configure the necessary camera parameters (e.g.,
  // baseline, focal length, etc.).
  PointCloudGenerator point_cloud_generator(p_cam);

  // Set the extents of the point cloud based on the camera node map
  // (node_map_camera), which contains information about the camera's image
  // resolution and other relevant metadata.
  point_cloud_generator.setPointCloudExtents(node_map_camera,
                                             point_cloud_parameters);

  // This object will hold the computed 3D point cloud, where each point
  // contains XYZ coordinates and RGB color values.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  // Create the ROS2 node for publishing images and point clouds.
  // The StereoImagePublisherNode will be responsible for capturing, processing,
  // and publishing images and point clouds.
  auto publisher_node = std::make_shared<StereoImagePublisherNode>(
      p_cam, camera_parameters, point_cloud_parameters, node_name);

  // Create a MultiThreadedExecutor, which allows running multiple callback
  // threads in ROS2.
  rclcpp::executors::MultiThreadedExecutor executor;

  // Add the publisher node (StereoImagePublisherNode) to the executor.
  executor.add_node(publisher_node);

  // Declare an ImageList object to hold the images captured by the camera.
  ImageList image_list;

  // Declare an ImagePtr object to hold the disparity image.
  ImagePtr disparity_image;

  // Define a polling interval (e.g., poll every 1 second)
  auto last_poll_time = std::chrono::steady_clock::now();
  std::chrono::seconds poll_interval(1);

  // Main loop that continues while ROS2 is running
  while (rclcpp::ok()) {
    if (p_cam->IsStreaming()) {
      // Capture the next set of images from the camera.
      // If the images can't be captured, log a warning and skip the iteration.
      uint64_t timeout_milli_secs =
          uint64_t((1000.0 / camera_parameters.frame_rate) * 2.0);
      if (!SpinStereo::get_next_image_set(
              p_cam, camera_parameters.stream_transmit_flags,
              timeout_milli_secs, image_list)) {
        RCLCPP_WARN(rclcpp::get_logger(node_name),
                    "Failed to get next image set.");
        continue;
      }

      // Apply post-processing on the disparity image if the corresponding flags
      // are enabled. Ensure the disparity image is not incomplete before
      // applying the filter.
      if (camera_parameters.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {
        if (camera_parameters.post_process_disparity) {
          // Apply a speckle filter to clean up the disparity image using the
          // provided parameters.
          disparity_image = ImageUtilityStereo::FilterSpeckles(
              image_list.GetByPayloadType(
                  SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1),
              camera_parameters.max_speckle_size, camera_parameters.max_diff,
              stereo_parameters.disparityScaleFactor,
              stereo_parameters.invalidDataValue);
        } else {
          // If no post-processing is needed, simply use the disparity image as
          // it is.
          disparity_image = image_list.GetByPayloadType(
              SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
        }
      }

      // If point cloud computation is enabled and the disparity image is valid:
      if (camera_parameters.do_compute_point_cloud &&
          camera_parameters.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {
        // Clear the existing point cloud to prepare for new data.
        point_cloud->clear();

        float min_disparity = camera_parameters.scan3d_coordinate_offset;

        // If the rectified image is available, use it to color the point cloud.
        if (camera_parameters.stream_transmit_flags
                .rect_sensor_1_transmit_enabled &&
            !image_list
                 .GetByPayloadType(
                     SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1)
                 ->IsIncomplete()) {
          point_cloud_generator.compute_point_cloud(
              disparity_image,
              image_list.GetByPayloadType(
                  SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1),
              min_disparity, stereo_parameters, point_cloud_parameters,
              point_cloud);
        } else {
          // If no rectified image is available, compute the point cloud without
          // coloring.
          point_cloud_generator.compute_point_cloud(
              disparity_image, min_disparity, stereo_parameters,
              point_cloud_parameters, point_cloud);
        }
      }

      // Get the current ROS2 time_stamp to add to the published messages.
      auto time_stamp = publisher_node->now();

      // Publish the raw left image if enabled and the image is complete.
      if (camera_parameters.stream_transmit_flags
              .raw_sensor_1_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1)
               ->IsIncomplete()) {
        publisher_node->publishImage(
            image_list.GetByPayloadType(
                SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1),
            publisher_node->getRawLeftPublisher(), "rgb8", time_stamp);
      }

      // Publish the raw right image if enabled and the image is complete.
      if (camera_parameters.stream_transmit_flags
              .raw_sensor_2_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2)
               ->IsIncomplete()) {
        publisher_node->publishImage(
            image_list.GetByPayloadType(
                SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2),
            publisher_node->getRawRightPublisher(), "rgb8", time_stamp);
      }

      // Publish the rectified left image if enabled and the image is complete.
      if (camera_parameters.stream_transmit_flags
              .rect_sensor_1_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1)
               ->IsIncomplete()) {
        publisher_node->publishImage(
            image_list.GetByPayloadType(
                SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1),
            publisher_node->getRectLeftPublisher(), "rgb8", time_stamp);
      }

      // Publish the rectified right image if enabled and the image is complete.
      if (camera_parameters.stream_transmit_flags
              .rect_sensor_2_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2)
               ->IsIncomplete()) {
        publisher_node->publishImage(
            image_list.GetByPayloadType(
                SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2),
            publisher_node->getRectRightPublisher(), "rgb8", time_stamp);
      }

      // Publish the disparity image if enabled and the image is complete.
      if (camera_parameters.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {
        publisher_node->publishDisparityImage(disparity_image, time_stamp);
      }

      // Publish the point cloud if enabled and the disparity image is
      // available.
      if (camera_parameters.do_compute_point_cloud) {
        if (camera_parameters.stream_transmit_flags
                .disparity_transmit_enabled &&
            !image_list
                 .GetByPayloadType(
                     SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
                 ->IsIncomplete()) {
          publisher_node->publishPointCloud(point_cloud, time_stamp);
        }
      }
    }

    if (!p_cam->IsValid()) {
      RCLCPP_ERROR(rclcpp::get_logger(node_name),
                   "The camera seems to be disconnected. Try reconnecting and "
                   "running the program again.");
      break;
    }

    // Check if it's time to poll the node map
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_poll_time >= poll_interval) {
      // Poll the camera’s node map for parameter changes from other sources
      publisher_node->poll_node_map_parameters(p_cam, camera_parameters,
                                               point_cloud_parameters);
      last_poll_time = current_time;
    }

    // Allow the executor to process any pending callbacks (e.g., parameter
    // updates).
    executor.spin_some();
  }

  try {
    // setting the device throughput to max
    set_device_link_throughput_to_max(p_cam);

    // End acquisition
    p_cam->EndAcquisition();

    // De-initialize the camera and release it
    p_cam->DeInit();
    p_cam = nullptr;  // Reset the camera pointer to null to avoid memory issues

  } catch (const Spinnaker::Exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger(node_name), "Exception during cleanup: %s",
                 e.what());
  }

  // Clear the list of cameras and release the system instance
  camList.Clear();
  system->ReleaseInstance();

  // Shutdown the ROS2 node and stop all ROS2 processes
  rclcpp::shutdown();

  return 0;
}
