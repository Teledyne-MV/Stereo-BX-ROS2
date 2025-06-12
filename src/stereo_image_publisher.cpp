//=============================================================================
// Copyright (c) 2024 FLIR Integrated Imaging Solutions, Inc.
// All Rights Reserved
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

#include <mutex>
#include <thread>

#include <cv_bridge/cv_bridge.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include "ImageUtilityStereo.h"
#include "PointCloud.h"
#include "Spinnaker.h"
#include "stereo_image_publisher/spin_stereo_helper.hpp"
#include "stereo_image_publisher/stereo_parameters.hpp"

using namespace std;
using namespace SpinStereo;
using namespace Spinnaker;
using namespace GenApi;

class PointCloudGenerator {
 public:
  PointCloudGenerator() {};

  bool setPointCloudExtents(INodeMap& node_map_,
                            PointCloudParameters& point_cloud_parameters_) {
    int64_t stereo_height, stereo_width;
    if (!get_int_value_from_node(node_map_, "StereoHeight", stereo_height)) {
      stereo_height = 1536;
    }
    if (!get_int_value_from_node(node_map_, "StereoWidth", stereo_width)) {
      stereo_width = 2048;
    }

    point_cloud_parameters_.decimationFactor = 4;
    point_cloud_parameters_.ROIImageLeft = 0;                      // U Min
    point_cloud_parameters_.ROIImageTop = 0;                       // V Min
    point_cloud_parameters_.ROIImageRight = (uint)stereo_width;    // U Max
    point_cloud_parameters_.ROIImageBottom = (uint)stereo_height;  // V Max

    // Region of Interest (ROI) in World Coordinates
    point_cloud_parameters_.ROIWorldCoordinatesXMin = -10.0f;  // X Min
    point_cloud_parameters_.ROIWorldCoordinatesXMax = 10.0f;   // X Max
    point_cloud_parameters_.ROIWorldCoordinatesYMin = -10.0f;  // Y Min
    point_cloud_parameters_.ROIWorldCoordinatesYMax = 10.0f;   // Y Max
    point_cloud_parameters_.ROIWorldCoordinatesZMin = 0.0f;    // Z Min
    point_cloud_parameters_.ROIWorldCoordinatesZMax = 20.0f;   // Z Max

    return true;
  }

  bool compute_point_cloud(const ImagePtr& disparity_image,
                           const ImagePtr& reference_image,
                           const StereoCameraParameters& stereo_parameters,
                           const PointCloudParameters& point_cloud_parameters,
                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud)
  {
    auto spin_pc = ImageUtilityStereo::ComputePointCloud(
                    disparity_image,
                    reference_image,
                    point_cloud_parameters,
                    stereo_parameters);

    // Convert to PCL
    point_cloud->clear();
    const size_t N = spin_pc.GetNumPoints();
    point_cloud->points.reserve(N);

    for (size_t i = 0; i < N; ++i) {
      auto pt = spin_pc.GetPoint(i);
      pcl::PointXYZRGB p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.r = pt.r;
      p.g = pt.g;
      p.b = pt.b;
      point_cloud->points.push_back(p);
    }
    return true;
  }
};

sensor_msgs::msg::CameraInfo generateCameraInfo(
    const StereoCameraParameters& stereo_params, int width, int height) {
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  info.distortion_model = "plumb_bob";
  info.d = {0.0, 0.0, 0.0, 0.0, 0.0};  // assuming no distortion or rectified

  info.k = {
    stereo_params.focalLength, 0.0, stereo_params.principalPointU,
    0.0, stereo_params.focalLength, stereo_params.principalPointV,
    0.0, 0.0, 1.0
  };

  info.r = {
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
  };

  info.p = {
    stereo_params.focalLength, 0.0, stereo_params.principalPointU, 0.0,
    0.0, stereo_params.focalLength, stereo_params.principalPointV, 0.0,
    0.0, 0.0, 1.0, 0.0
  };

  return info;
}


class StereoImagePublisherNode : public rclcpp::Node {
 public:
  StereoImagePublisherNode(const rclcpp::NodeOptions & options) : Node("stereo_image_publisher_node") {

    declare_serial_number_parameter();

    system = System::GetInstance();
    cam_list = system->GetCameras();
    // Initialize camera first to access node_map_
    if (!initialize_camera()) {
      RCLCPP_FATAL(this->get_logger(),
                   "Failed to initialize the camera. Shutting down.");
      rclcpp::shutdown();
      return;
    }

    configure_pixel_format();

    configure_resolution();

    // Declare parameters with constraints based on camera's capabilities
    declare_parameters();

    // Initialize publishers based on current stream flags
    initialize_publishers();

    // Set up dynamic parameter callback
    parameter_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&StereoImagePublisherNode::parametersCallback, this,
                  std::placeholders::_1));

    // Set up timer for image acquisition
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(
            static_cast<int>(1000.0 / camera_parameters_.frame_rate)),
        std::bind(&StereoImagePublisherNode::timer_callback, this));

    // Initialize the polling timer to call poll_node_map_parameters every 1
    // second
    poll_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&StereoImagePublisherNode::poll_node_map_parameters, this));
  }

  ~StereoImagePublisherNode() {
    RCLCPP_INFO(this->get_logger(),
                "Destructor called. Initiating shutdown sequence.");

    // Cancel the timer to stop further callbacks
    if (timer_) {
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Timer cancelled.");
    }

    // **Cancel the polling timer**
    if (poll_timer_) {
      poll_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Polling timer cancelled.");
    }

    // Reset publishers first to release any references to camera interfaces
    raw_left_image_publisher_.reset();
    raw_right_image_publisher_.reset();
    rect_left_image_publisher_.reset();
    rect_right_image_publisher_.reset();
    disparity_image_publisher_.reset();
    point_cloud_publisher_.reset();
    RCLCPP_INFO(this->get_logger(), "Publishers reset.");

    // Lock the mutex before accessing camera resources
    {
      std::lock_guard<std::mutex> lock(camera_mutex_);
      if (p_cam_ && p_cam_->IsStreaming()) {
        try {
          p_cam_->EndAcquisition();
          p_cam_->DeInit();
          RCLCPP_INFO(this->get_logger(),
                      "Camera acquisition ended and deinitialized.");
        } catch (const Spinnaker::Exception& e) {
          RCLCPP_ERROR(this->get_logger(),
                       "Exception during camera shutdown: %s", e.what());
        }
      }

      // Reset node_map_ pointer
      node_map_ = nullptr;
    }

    // Optionally, release the Spinnaker system instance if no cameras are left
    System::GetInstance()->ReleaseInstance();

    RCLCPP_INFO(this->get_logger(), "Destructor completed. Node shutdown.");
  }

 private:
  // ROS 2 Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      raw_left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      raw_right_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      rect_left_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      rect_right_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      rect_left_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr
      rect_right_camera_info_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
      disparity_image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_publisher_;

  std::mutex camera_mutex_;

  // Camera
  SystemPtr system;
  CameraList cam_list;
  CameraPtr p_cam_;
  string serial_number_;
  INodeMap* node_map_;

  // Camera Parameters
  StereoParameters camera_parameters_;

  // Point cloud parameters
  PointCloudParameters point_cloud_parameters_;

  // Stereo parameters
  StereoCameraParameters stereo_parameters_;

  // The GenApi enumeration node for PixelFormat:
  GenApi::CEnumerationPtr pixel_format_enum_;

  // The list of symbolic names, e.g. {"RGB8","Mono8", …}
  std::vector<std::string> pixel_format_options_;

  // A reusable description builder (optional)
  std::ostringstream pixel_format_desc_;

  bool do_update_pixel_format_{false};

  std::vector<std::string> resolution_options_;  
  std::ostringstream resolution_desc_;  
  bool do_update_resolution_{false};

  // Point cloud generator
  PointCloudGenerator point_cloud_generator;

  // ROS 2 Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // NodeMap polling Timer
  rclcpp::TimerBase::SharedPtr poll_timer_;

  // Parameter callback handle
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  // Initialize and configure the camera
  bool initialize_camera() {
    try {
      // Initialize system and get camera list
      const unsigned int num_cameras = cam_list.GetSize();

      if (cam_list.GetSize() == 0) {
        RCLCPP_ERROR(this->get_logger(), "No cameras detected.");
        return false;
      }

      if (camera_parameters_.serial_number != "") {
        // Log the retrieved serial number
        RCLCPP_INFO(this->get_logger(),
                    "The camera serial number we are looking for is: %s",
                    camera_parameters_.serial_number.c_str());

        // Select the camera based on serial number
        for (unsigned int i = 0; i < num_cameras; ++i) {
          CameraPtr cam = cam_list.GetByIndex(i);
          std::string camera_serial =
              cam->TLDevice.DeviceSerialNumber.GetValue().c_str();
          if (camera_serial == camera_parameters_.serial_number) {
            // Check to make sure camera supports stereo vision
            RCLCPP_DEBUG(rclcpp::get_logger("stereo_image_publisher"),
                         "Checking camera stereo support...");
            if (!ImageUtilityStereo::IsStereoCamera(cam)) {
              RCLCPP_ERROR(
                  rclcpp::get_logger("stereo_image_publisher"),
                  "Camera with serial number %s is not a valid BumbleBee "
                  "X camera. Abborting...",
                  cam->TLDevice.DeviceSerialNumber.GetValue().c_str());
              cam_list.Clear();  // Clear the camera list to release resources
              system
                  ->ReleaseInstance();  // Release the Spinnaker system instance
              return -1;  // Return error code as no cameras are available
            }
            p_cam_ = cam;
            serial_number_ = camera_serial;
            RCLCPP_INFO(this->get_logger(),
                        "Using camera with serial number: %s",
                        serial_number_.c_str());
            break;
          }
        }
        if (!p_cam_) {
          RCLCPP_ERROR(this->get_logger(),
                       "Specified camera not found. Aborting.");
          return false;
        }
      } else {
        // If the camera serial number is not defined, cycle through cameras to
        // get the first BumbleBee X.
        for (unsigned int i = 0; i < num_cameras; ++i) {
          CameraPtr cam = cam_list.GetByIndex(i);
          if (!ImageUtilityStereo::IsStereoCamera(cam)) {
            RCLCPP_DEBUG(
                rclcpp::get_logger("stereo_image_publisher"),
                "Camera with serial number %s is not a valid BumbleBee X "
                "camera. Checking other cameras...",
                cam->TLDevice.DeviceSerialNumber.GetValue().c_str());
            continue;
          }
          p_cam_ = cam;
          serial_number_ =
              p_cam_->TLDevice.DeviceSerialNumber.GetValue().c_str();
          break;
        }

        camera_parameters_.serial_number = serial_number_;
        this->set_parameter(rclcpp::Parameter("camera.serial_number", serial_number_));
        RCLCPP_INFO(this->get_logger(),
                          "Camera Serial Number: %s",
                          camera_parameters_.serial_number.c_str());
        // If no matching camera was found, release resources and return an
        // error
        if (!p_cam_) {
          RCLCPP_ERROR(this->get_logger(),
                       "No BumbleBee X detected. Aborting.");
          return false;
        }
      }

      // Initialize the camera
      p_cam_->Init();

      // Configure camera settings using helper functions
      if (!configure_camera()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to configure camera acquisition settings.");
        return false;
      }

      // Initialize stereo processing parameters
      if (!configure_stereo_processing(p_cam_->GetNodeMap(),
                                       stereo_parameters_)) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to configure stereo processing parameters.");
        return false;
      }

      // Print camera calibration and SGBM parameters for verification
      // print_device_info(p_cam_->GetNodeMap());
      // print_camera_calibration_params(p_cam_->GetNodeMap());
      // print_SGBM_params(p_cam_);
      point_cloud_generator.setPointCloudExtents(p_cam_->GetNodeMap(),
                                                 point_cloud_parameters_);

      if (!p_cam_->IsStreaming()) {
        RCLCPP_INFO(this->get_logger(), "Begin acquisition...");
        p_cam_->BeginAcquisition();
      }

      // Getting the node map
      node_map_ = &(p_cam_->GetNodeMap());

      return true;
    } catch (const Spinnaker::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Exception during camera initialization: %s", e.what());
      return false;
    }
  }

  bool configure_camera() {
    // Configure camera acquisition and processing settings

    if (!configure_acquisition(p_cam_,
                               camera_parameters_.stream_transmit_flags)) {
      RCLCPP_ERROR(this->get_logger(), "Unable to configure camera streams.");
      return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Configuring device link throughput...");
    if (!set_device_link_throughput_to_max(p_cam_)) {
      RCLCPP_WARN(this->get_logger(), "Unable to set device link throughput.");
    }

    RCLCPP_DEBUG(this->get_logger(), "Setting acquisition frame rate...");
    if (!set_frame_rate(p_cam_, camera_parameters_.frame_rate)) {
      RCLCPP_WARN(this->get_logger(), "Unable to set acquisition frame rate.");
    } else {
      if (!get_frame_rate(p_cam_, camera_parameters_.frame_rate)) {
        RCLCPP_WARN(this->get_logger(),
                    "Unable to get acquisition frame rate.");
      } else {
        RCLCPP_DEBUG(this->get_logger(), "frame_rate updated: %f",
                     camera_parameters_.frame_rate);
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Setting pixel format...");
    if (camera_parameters_.pixel_format == "") {
      get_pixel_format(p_cam_, camera_parameters_.pixel_format);
    } else {
      if (!set_pixel_format(p_cam_, camera_parameters_.pixel_format)) {
        RCLCPP_WARN(this->get_logger(), "Unable to set pixel format to %s.",
                    camera_parameters_.pixel_format.c_str());
      } else {
        if (!get_pixel_format(p_cam_, camera_parameters_.pixel_format)) {
          RCLCPP_WARN(this->get_logger(),
                      "Unable to get acquisition frame rate.");
        } else {
          RCLCPP_DEBUG(this->get_logger(), "pixel_format updated: %s",
                      camera_parameters_.pixel_format.c_str());
        }
      }
    }

    RCLCPP_DEBUG(this->get_logger(), "Configuring device link throughput...");
    if (!set_device_link_throughput_to_current(p_cam_)) {
      RCLCPP_WARN(this->get_logger(), "Unable to set device link throughput.");
    }

    return true;
  }

  void configure_pixel_format()
  {
    CEnumerationPtr ptr_source_selector = node_map_->GetNode("SourceSelector");
    CEnumerationPtr ptr_component_selector =
        node_map_->GetNode("ComponentSelector");
        
    GenApi::CEnumEntryPtr ptr_source_sensor_1 =
        ptr_source_selector->GetEntryByName("Sensor1");
    GenApi::CEnumEntryPtr ptr_component_rectified =
        ptr_component_selector->GetEntryByName("Rectified");

    // Raw Sensor1
    ptr_source_selector->SetIntValue(ptr_source_sensor_1->GetValue());
    ptr_component_selector->SetIntValue(ptr_component_rectified->GetValue());
    
        // 1) Grab the generic node pointer…
    GenApi::INode* node =
      node_map_->GetNode("PixelFormat");
    // …and cast it to an IEnumeration
    pixel_format_enum_ =
      static_cast<GenApi::CEnumerationPtr>(node);

    // 2) Check availability/read-access
    if (!GenApi::IsAvailable(pixel_format_enum_) ||
        !GenApi::IsReadable (pixel_format_enum_))
    {
      RCLCPP_WARN(get_logger(),
                  "PixelFormat not available/readable");
      return;
    }

    // 3) Iterate its entries
    pixel_format_options_.clear();
    GenApi::NodeList_t entries;
    pixel_format_enum_->GetEntries(entries);

    for (auto &n : entries)
    {
      auto e = static_cast<GenApi::CEnumEntryPtr>(n);
      if (!GenApi::IsAvailable(e) || !GenApi::IsReadable(e))
        continue;

      // Convert gcstring → std::string
      pixel_format_options_.push_back(std::string(e->GetSymbolic().c_str()));
    }

    // 4) Build your human-readable description
    pixel_format_desc_.str("");
    pixel_format_desc_.clear();
    pixel_format_desc_ << "Choose pixel_format from: ";
    for (size_t i = 0; i < pixel_format_options_.size(); ++i)
    {
      pixel_format_desc_ << pixel_format_options_[i]
                        << (i+1 < pixel_format_options_.size() ? ", " : "");
    }

    RCLCPP_INFO(get_logger(),
                "Supported PixelFormats: %s",
                pixel_format_desc_.str().c_str());
  }

  void configure_resolution()
  {
    // 1) Grab the GenICam enumeration node
    GenApi::INode* node = node_map_->GetNode("StereoResolution");
    auto resolution_enum = static_cast<GenApi::CEnumerationPtr>(node);

    // 2) Check availability/read‐access
    if (!GenApi::IsAvailable(resolution_enum) || !GenApi::IsReadable(resolution_enum))
    {
      RCLCPP_WARN(get_logger(), "StereoResolution not available/readable");
      return;
    }

    // 3) Iterate its symbolic entries
    resolution_options_.clear();
    GenApi::NodeList_t entries;
    resolution_enum->GetEntries(entries);

    for (auto &n : entries)
    {
      auto e = static_cast<GenApi::CEnumEntryPtr>(n);
      if (!GenApi::IsAvailable(e) || !GenApi::IsReadable(e))
        continue;
      resolution_options_.push_back(std::string(e->GetSymbolic().c_str()));
    }

    // 4) Build the human‐readable description
    resolution_desc_.str("");
    resolution_desc_.clear();
    resolution_desc_ << "Choose StereoResolution from: ";
    for (size_t i = 0; i < resolution_options_.size(); ++i)
    {
      resolution_desc_ << resolution_options_[i]
                      << (i + 1 < resolution_options_.size() ? ", " : "");
    }

    // 5) Log it so RQT’s declare_parameters() can use it
    RCLCPP_INFO(get_logger(),
                "Supported StereoResolutions: %s",
                resolution_desc_.str().c_str());
  }


  void declare_serial_number_parameter() {
    // Declare the serial_number parameter with a default empty string
    this->declare_parameter<std::string>("camera.serial_number", "");

    // Retrieve the serial_number parameter value
    this->get_parameter("camera.serial_number", camera_parameters_.serial_number);
  }


  // Declare and initialize ROS 2 parameters with constraints
  void declare_parameters() {
    // Stereo Parameters
    // stereo.min_disparity
    rcl_interfaces::msg::ParameterDescriptor minDisp_desc;
    float minScan3dCoordinateOffset, maxScan3dCoordinateOffset;
    if (SpinStereo::get_min_float_value_from_node(
            *node_map_, "Scan3dCoordinateOffset", minScan3dCoordinateOffset) &&
        SpinStereo::get_max_float_value_from_node(
            *node_map_, "Scan3dCoordinateOffset", maxScan3dCoordinateOffset)) {
      minDisp_desc.description =
          "Minimum Scan3d coordinate offset (min disparity), clamped between " +
          std::to_string(minScan3dCoordinateOffset) + " and " +
          std::to_string(maxScan3dCoordinateOffset);
      minDisp_desc.floating_point_range.emplace_back(
          rcl_interfaces::msg::FloatingPointRange());
      minDisp_desc.floating_point_range.back().from_value =
          minScan3dCoordinateOffset;
      minDisp_desc.floating_point_range.back().to_value =
          maxScan3dCoordinateOffset;
      this->declare_parameter<double>(
          "stereo.min_disparity", camera_parameters_.scan3d_coordinate_offset,
          minDisp_desc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for 'stereo.min_disparity'. "
                  "Declaring without constraints.");
      this->declare_parameter<double>(
          "stereo.min_disparity", camera_parameters_.scan3d_coordinate_offset);
    }

    // stereo.small_penalty
    rcl_interfaces::msg::ParameterDescriptor smallPenalty_desc;
    int64_t minSmallPenalty, maxSmallPenalty;
    if (SpinStereo::get_min_int_value_from_node(*node_map_, "SmallPenalty",
                                                minSmallPenalty) &&
        SpinStereo::get_max_int_value_from_node(*node_map_, "SmallPenalty",
                                                maxSmallPenalty)) {
      smallPenalty_desc.description = "Small penalty value, clamped between " +
                                      std::to_string(minSmallPenalty) +
                                      " and " + std::to_string(maxSmallPenalty);
      smallPenalty_desc.integer_range.emplace_back(
          rcl_interfaces::msg::IntegerRange());
      smallPenalty_desc.integer_range.back().from_value = minSmallPenalty;
      smallPenalty_desc.integer_range.back().to_value = maxSmallPenalty;
      this->declare_parameter<int>("stereo.small_penalty",
                                   camera_parameters_.small_penalty,
                                   smallPenalty_desc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for 'stereo.small_penalty'. "
                  "Declaring without constraints.");
      this->declare_parameter<int>("stereo.small_penalty",
                                   camera_parameters_.small_penalty);
    }

    // stereo.large_penalty
    rcl_interfaces::msg::ParameterDescriptor largePenalty_desc;
    int64_t minLargePenalty, maxLargePenalty;
    if (SpinStereo::get_min_int_value_from_node(*node_map_, "LargePenalty",
                                                minLargePenalty) &&
        SpinStereo::get_max_int_value_from_node(*node_map_, "LargePenalty",
                                                maxLargePenalty)) {
      largePenalty_desc.description = "Large penalty value, clamped between " +
                                      std::to_string(minLargePenalty) +
                                      " and " + std::to_string(maxLargePenalty);
      largePenalty_desc.integer_range.emplace_back(
          rcl_interfaces::msg::IntegerRange());
      largePenalty_desc.integer_range.back().from_value = minLargePenalty;
      largePenalty_desc.integer_range.back().to_value = maxLargePenalty;
      this->declare_parameter<int>("stereo.large_penalty",
                                   camera_parameters_.large_penalty,
                                   largePenalty_desc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for 'stereo.large_penalty'. "
                  "Declaring without constraints.");
      this->declare_parameter<int>("stereo.large_penalty",
                                   camera_parameters_.large_penalty);
    }

    // stereo.uniqueness_ratio
    rcl_interfaces::msg::ParameterDescriptor uniquenessRatio_desc;
    int64_t minUniquenessRatio, maxUniquenessRatio;
    if (SpinStereo::get_min_int_value_from_node(*node_map_, "UniquenessRatio",
                                                minUniquenessRatio) &&
        SpinStereo::get_max_int_value_from_node(*node_map_, "UniquenessRatio",
                                                maxUniquenessRatio)) {
      uniquenessRatio_desc.description = "Uniqueness ratio, clamped between " +
                                         std::to_string(minUniquenessRatio) +
                                         " and " +
                                         std::to_string(maxUniquenessRatio);
      uniquenessRatio_desc.integer_range.emplace_back(
          rcl_interfaces::msg::IntegerRange());
      uniquenessRatio_desc.integer_range.back().from_value = minUniquenessRatio;
      uniquenessRatio_desc.integer_range.back().to_value = maxUniquenessRatio;
      this->declare_parameter<int>("stereo.uniqueness_ratio",
                                   camera_parameters_.uniqueness_ratio,
                                   uniquenessRatio_desc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for "
                  "'stereo.uniqueness_ratio'. Declaring without constraints.");
      this->declare_parameter<int>("stereo.uniqueness_ratio",
                                   camera_parameters_.uniqueness_ratio);
    }

    // Set up a read-only parameter descriptor
    rcl_interfaces::msg::ParameterDescriptor read_only_descriptor;
    // read_only_descriptor.read_only = true;

    // Declare the parameters as read-only
    this->declare_parameter<double>("stereo.principal_point_u",
                                    stereo_parameters_.principalPointU,
                                    read_only_descriptor);
    this->declare_parameter<double>("stereo.principal_point_v",
                                    stereo_parameters_.principalPointV,
                                    read_only_descriptor);
    this->declare_parameter<double>("stereo.scale_factor",
                                    stereo_parameters_.disparityScaleFactor,
                                    read_only_descriptor);
    this->declare_parameter<double>(
        "stereo.baseline", stereo_parameters_.baseline, read_only_descriptor);
    this->declare_parameter<double>("stereo.focal_length",
                                    stereo_parameters_.focalLength,
                                    read_only_descriptor);

    // Post Processing Parameters
    this->declare_parameter<bool>("post_processing.enable",
                                  camera_parameters_.post_process_disparity);
    this->declare_parameter<int>("post_processing.max_speckle_size",
                                 camera_parameters_.max_speckle_size);
    this->declare_parameter<double>("post_processing.max_diff",
                                    camera_parameters_.max_diff);

    // Stream Transmit Flags
    // stream.raw_left_enabled
    rcl_interfaces::msg::ParameterDescriptor rawLeftDesc;
    rawLeftDesc.description =
        "Enable or disable transmission of raw left sensor images.";
    this->declare_parameter<bool>(
        "stream.raw_left_enabled",
        camera_parameters_.stream_transmit_flags.raw_sensor_1_transmit_enabled,
        rawLeftDesc);

    // stream.raw_right_enabled
    rcl_interfaces::msg::ParameterDescriptor rawRightDesc;
    rawRightDesc.description =
        "Enable or disable transmission of raw right sensor images.";
    this->declare_parameter<bool>(
        "stream.raw_right_enabled",
        camera_parameters_.stream_transmit_flags.raw_sensor_2_transmit_enabled,
        rawRightDesc);

    // stream.rect_left_enabled
    rcl_interfaces::msg::ParameterDescriptor rectLeftDesc;
    rectLeftDesc.description =
        "Enable or disable transmission of rectified left sensor images.";
    this->declare_parameter<bool>(
        "stream.rect_left_enabled",
        camera_parameters_.stream_transmit_flags.rect_sensor_1_transmit_enabled,
        rectLeftDesc);

    // stream.rect_right_enabled
    rcl_interfaces::msg::ParameterDescriptor rectRightDesc;
    rectRightDesc.description =
        "Enable or disable transmission of rectified right sensor images.";
    this->declare_parameter<bool>(
        "stream.rect_right_enabled",
        camera_parameters_.stream_transmit_flags.rect_sensor_2_transmit_enabled,
        rectRightDesc);

    // stream.disparity_enabled
    rcl_interfaces::msg::ParameterDescriptor disparityDesc;
    disparityDesc.description =
        "Enable or disable transmission of disparity images.";
    this->declare_parameter<bool>(
        "stream.disparity_enabled",
        camera_parameters_.stream_transmit_flags.disparity_transmit_enabled,
        disparityDesc);

    // Point Cloud Parameters
    // point_cloud.enable
    rcl_interfaces::msg::ParameterDescriptor pointCloudEnableDesc;
    pointCloudEnableDesc.description =
        "Enable or disable point cloud computation.";
    this->declare_parameter<bool>("point_cloud.enable",
                                  camera_parameters_.do_compute_point_cloud,
                                  pointCloudEnableDesc);

    // point_cloud.decimation_factor
    rcl_interfaces::msg::ParameterDescriptor decimationFactorDesc;
    int64_t minDecimationFactor = 1;
    int64_t maxDecimationFactor = 100;  // Arbitrary upper limit
    decimationFactorDesc.description =
        "Decimation factor for point cloud generation, clamped between " +
        std::to_string(minDecimationFactor) + " and " +
        std::to_string(maxDecimationFactor);
    decimationFactorDesc.integer_range.emplace_back(
        rcl_interfaces::msg::IntegerRange());
    decimationFactorDesc.integer_range.back().from_value = minDecimationFactor;
    decimationFactorDesc.integer_range.back().to_value = maxDecimationFactor;
    this->declare_parameter<int>("point_cloud.decimation_factor",
                                 point_cloud_parameters_.decimationFactor,
                                 decimationFactorDesc);

    // point_cloud.XMin
    rcl_interfaces::msg::ParameterDescriptor XMinDesc;
    XMinDesc.description =
        "Minimum X coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.XMin", point_cloud_parameters_.ROIWorldCoordinatesXMin,
        XMinDesc);

    // point_cloud.XMax
    rcl_interfaces::msg::ParameterDescriptor XMaxDesc;
    XMaxDesc.description =
        "Maximum X coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.XMax", point_cloud_parameters_.ROIWorldCoordinatesXMax,
        XMaxDesc);

    // point_cloud.YMin
    rcl_interfaces::msg::ParameterDescriptor YMinDesc;
    YMinDesc.description =
        "Minimum Y coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.YMin", point_cloud_parameters_.ROIWorldCoordinatesYMin,
        YMinDesc);

    // point_cloud.YMax
    rcl_interfaces::msg::ParameterDescriptor YMaxDesc;
    YMaxDesc.description =
        "Maximum Y coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.YMax", point_cloud_parameters_.ROIWorldCoordinatesYMax,
        YMaxDesc);

    // point_cloud.ZMin
    rcl_interfaces::msg::ParameterDescriptor ZMinDesc;
    ZMinDesc.description =
        "Minimum Z coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.ZMin", point_cloud_parameters_.ROIWorldCoordinatesZMin,
        ZMinDesc);

    // point_cloud.ZMax
    rcl_interfaces::msg::ParameterDescriptor ZMaxDesc;
    ZMaxDesc.description =
        "Maximum Z coordinate in world frame for point cloud generation.";
    this->declare_parameter<float>(
        "point_cloud.ZMax", point_cloud_parameters_.ROIWorldCoordinatesZMax,
        ZMaxDesc);

    int64_t stereo_height_max, stereo_width_max;
    if (!get_int_value_from_node(*node_map_, "HeightMax", stereo_height_max)) {
      stereo_height_max = 1536;
    }
    if (!get_int_value_from_node(*node_map_, "WidthMax", stereo_width_max)) {
      stereo_width_max = 2048;
    }

    // point_cloud.ROI_image_left
    rcl_interfaces::msg::ParameterDescriptor ROILeftDesc;
    int minROIImageLeft = 0;
    int maxROIImageLeft = stereo_width_max;
    ROILeftDesc.description = "Left boundary of ROI image, clamped between " +
                              std::to_string(minROIImageLeft) + " and " +
                              std::to_string(maxROIImageLeft);
    ROILeftDesc.integer_range.emplace_back(rcl_interfaces::msg::IntegerRange());
    ROILeftDesc.integer_range.back().from_value = minROIImageLeft;
    ROILeftDesc.integer_range.back().to_value = maxROIImageLeft;
    this->declare_parameter<int>("point_cloud.ROI_image_left",
                                 point_cloud_parameters_.ROIImageLeft,
                                 ROILeftDesc);

    // point_cloud.ROI_image_top
    rcl_interfaces::msg::ParameterDescriptor ROITopDesc;
    int minROITop = 0;
    int maxROITop = stereo_height_max;
    ROITopDesc.description = "Top boundary of ROI image, clamped between " +
                             std::to_string(minROITop) + " and " +
                             std::to_string(maxROITop);
    ROITopDesc.integer_range.emplace_back(rcl_interfaces::msg::IntegerRange());
    ROITopDesc.integer_range.back().from_value = minROITop;
    ROITopDesc.integer_range.back().to_value = maxROITop;
    this->declare_parameter<int>("point_cloud.ROI_image_top",
                                 point_cloud_parameters_.ROIImageTop,
                                 ROITopDesc);

    // point_cloud.ROI_image_right
    rcl_interfaces::msg::ParameterDescriptor ROIRightDesc;
    int minROIRight = 0;
    int maxROIRight = stereo_width_max;
    ROIRightDesc.description = "Right boundary of ROI image, clamped between " +
                               std::to_string(minROIRight) + " and " +
                               std::to_string(maxROIRight);
    ROIRightDesc.integer_range.emplace_back(
        rcl_interfaces::msg::IntegerRange());
    ROIRightDesc.integer_range.back().from_value = minROIRight;
    ROIRightDesc.integer_range.back().to_value = maxROIRight;
    this->declare_parameter<int>("point_cloud.ROI_image_right",
                                 point_cloud_parameters_.ROIImageRight,
                                 ROIRightDesc);

    // point_cloud.ROI_image_bottom
    rcl_interfaces::msg::ParameterDescriptor ROIBottomDesc;
    int minROIBottom = 0;
    int maxROIBottom = stereo_height_max;
    ROIBottomDesc.description =
        "Bottom boundary of ROI image, clamped between " +
        std::to_string(minROIBottom) + " and " + std::to_string(maxROIBottom);
    ROIBottomDesc.integer_range.emplace_back(
        rcl_interfaces::msg::IntegerRange());
    ROIBottomDesc.integer_range.back().from_value = minROIBottom;
    ROIBottomDesc.integer_range.back().to_value = maxROIBottom;
    this->declare_parameter<int>("point_cloud.ROI_image_bottom",
                                 point_cloud_parameters_.ROIImageBottom,
                                 ROIBottomDesc);

    // Camera Parameters
    // camera.auto_exposure
    rcl_interfaces::msg::ParameterDescriptor autoExposureDesc;
    autoExposureDesc.description = "Enable or disable automatic exposure.";
    this->declare_parameter<bool>("camera.auto_exposure",
                                  camera_parameters_.auto_exposure,
                                  autoExposureDesc);

    // camera.exposure_time
    rcl_interfaces::msg::ParameterDescriptor exposureTimeDesc;
    float minExposureTime, maxExposureTime;
    if (SpinStereo::get_min_float_value_from_node(*node_map_, "ExposureTime",
                                                  minExposureTime) &&
        SpinStereo::get_max_float_value_from_node(*node_map_, "ExposureTime",
                                                  maxExposureTime)) {
      exposureTimeDesc.description =
          "Exposure time in microseconds, clamped between " +
          std::to_string(minExposureTime) + " and " +
          std::to_string(maxExposureTime);
      exposureTimeDesc.floating_point_range.emplace_back(
          rcl_interfaces::msg::FloatingPointRange());
      exposureTimeDesc.floating_point_range.back().from_value = minExposureTime;
      exposureTimeDesc.floating_point_range.back().to_value = maxExposureTime;
      this->declare_parameter<double>("camera.exposure_time",
                                      camera_parameters_.exposure_time,
                                      exposureTimeDesc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for 'camera.exposure_time'. "
                  "Declaring without constraints.");
      this->declare_parameter<double>("camera.exposure_time",
                                      camera_parameters_.exposure_time);
    }

    // camera.auto_gain
    rcl_interfaces::msg::ParameterDescriptor autoGainDesc;
    autoGainDesc.description = "Enable or disable automatic gain.";
    this->declare_parameter<bool>("camera.auto_gain",
                                  camera_parameters_.auto_gain, autoGainDesc);

    // camera.gain_value
    rcl_interfaces::msg::ParameterDescriptor gainValueDesc;
    float minGain, maxGain;
    if (SpinStereo::get_min_float_value_from_node(*node_map_, "Gain",
                                                  minGain) &&
        SpinStereo::get_max_float_value_from_node(*node_map_, "Gain",
                                                  maxGain)) {
      gainValueDesc.description = "Gain value, clamped between " +
                                  std::to_string(minGain) + " and " +
                                  std::to_string(maxGain);
      gainValueDesc.floating_point_range.emplace_back(
          rcl_interfaces::msg::FloatingPointRange());
      gainValueDesc.floating_point_range.back().from_value = minGain;
      gainValueDesc.floating_point_range.back().to_value = maxGain;
      this->declare_parameter<double>(
          "camera.gain_value", camera_parameters_.gain_value, gainValueDesc);
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unable to retrieve min and max for 'camera.gain_value'. "
                  "Declaring without constraints.");
      this->declare_parameter<double>("camera.gain_value",
                                      camera_parameters_.gain_value);
    }

    // camera.acquisition_frame_rate_enabled
    rcl_interfaces::msg::ParameterDescriptor frameRateEnableDesc;
    frameRateEnableDesc.description =
        "Enable or disable acquisition frame rate control.";
    this->declare_parameter<bool>(
        "camera.acquisition_frame_rate_enabled",
        camera_parameters_.acquisition_frame_rate_enabled, frameRateEnableDesc);
    this->declare_parameter<double>("camera.frame_rate",
                                    camera_parameters_.frame_rate);

    // camera.pixel_format
    std::ostringstream json_enum;
    json_enum << "{\"enum\":[";
    for (size_t i = 0; i < pixel_format_options_.size(); ++i) {
      json_enum << "\"" << pixel_format_options_[i] << "\""
                << (i + 1 < pixel_format_options_.size() ? "," : "");
    }
    json_enum << "]}";

    rcl_interfaces::msg::ParameterDescriptor pixelFormatDesc;
    pixelFormatDesc.description = pixel_format_desc_.str();
    pixelFormatDesc.additional_constraints = json_enum.str();

    this->declare_parameter<std::string>(
        "camera.pixel_format",
        camera_parameters_.pixel_format,
        pixelFormatDesc);

    // camera.resolution
    rcl_interfaces::msg::ParameterDescriptor resolutionDesc;
    resolutionDesc.description            = resolution_desc_.str();
    resolutionDesc.additional_constraints =
      "{\"enum\": ["
        + std::accumulate(
            std::next(resolution_options_.begin()), resolution_options_.end(),
            std::string("\"") + resolution_options_[0] + "\"",
            [](auto a, auto &b){ return a + ",\"" + b + "\""; })
        + "]}";

    this->declare_parameter<std::string>(
      "camera.resolution",
      camera_parameters_.resolution,
      resolutionDesc);

  }

  // Initialize ROS 2 publishers based on stream flags
  void initialize_publishers() {
    // Reset all existing publishers before reinitializing
    raw_left_image_publisher_.reset();
    raw_right_image_publisher_.reset();
    rect_left_image_publisher_.reset();
    rect_right_image_publisher_.reset();
    disparity_image_publisher_.reset();
    point_cloud_publisher_.reset();
    rect_left_camera_info_publisher_.reset();
    rect_right_camera_info_publisher_.reset();

    // Get the camera's serial number
    std::string serial_number =
        "serial_" +
        std::string(p_cam_->TLDevice.DeviceSerialNumber.GetValue().c_str());

    // Conditionally create publishers based on the transmission flags
    if (camera_parameters_.stream_transmit_flags
            .raw_sensor_1_transmit_enabled) {
      raw_left_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/raw_left_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Raw left image publisher created.");
    }

    if (camera_parameters_.stream_transmit_flags
            .raw_sensor_2_transmit_enabled) {
      raw_right_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/raw_right_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Raw right image publisher created.");
    }

    if (camera_parameters_.stream_transmit_flags
            .rect_sensor_1_transmit_enabled) {
      rect_left_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/rectified_left_image",
              rclcpp::QoS(10).transient_local());
      rect_left_camera_info_publisher_ =
          this->create_publisher<sensor_msgs::msg::CameraInfo>(
              "Bumblebee_X/" + serial_number + "/left_camera_info",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(),
                  "Rectified left image publisher created.");
    }

    if (camera_parameters_.stream_transmit_flags
            .rect_sensor_2_transmit_enabled) {
      rect_right_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/rectified_right_image",
              rclcpp::QoS(10).transient_local());
      rect_right_camera_info_publisher_ =
          this->create_publisher<sensor_msgs::msg::CameraInfo>(
              "Bumblebee_X/" + serial_number + "/right_camera_info",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(),
                  "Rectified right image publisher created.");
    }

    if (camera_parameters_.stream_transmit_flags.disparity_transmit_enabled) {
      disparity_image_publisher_ =
          this->create_publisher<sensor_msgs::msg::Image>(
              "Bumblebee_X/" + serial_number + "/disparity_image",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Disparity image publisher created.");
    }

    if (camera_parameters_.do_compute_point_cloud &&
        camera_parameters_.stream_transmit_flags.disparity_transmit_enabled) {
      point_cloud_publisher_ =
          this->create_publisher<sensor_msgs::msg::PointCloud2>(
              "Bumblebee_X/" + serial_number + "/point_cloud",
              rclcpp::QoS(10).transient_local());
      RCLCPP_INFO(this->get_logger(), "Point cloud publisher created.");
    }
  }

  // Parameter callback function for dynamic reconfiguration
  rcl_interfaces::msg::SetParametersResult parametersCallback(
      const std::vector<rclcpp::Parameter>& params) {
    // Acquire the mutex at the beginning
    std::lock_guard<std::mutex> lock(camera_mutex_);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    bool updatePublisher = false;

    for (const auto& param : params) {
      const std::string& name = param.get_name();

      if (name == "stereo.min_disparity") {
        double new_value_double = param.as_double();
        float new_value = static_cast<float>(new_value_double);
        if (!SpinStereo::set_scan3d_coordinate_offset(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the Scan3dCoordinateOffset node.");
          result.successful = false;
        } else {
          camera_parameters_.scan3d_coordinate_offset = new_value;
          RCLCPP_INFO(this->get_logger(), "min_disparity updated: %f",
                      camera_parameters_.scan3d_coordinate_offset);
        }
      } else if (name == "stereo.small_penalty") {
        int new_value = param.as_int();
        if (new_value >= camera_parameters_.large_penalty) {
          RCLCPP_WARN(
              this->get_logger(),
              "small_penalty must be less than large_penalty. Clamping value.");
          new_value = camera_parameters_.large_penalty - 1;
        }
        if (!SpinStereo::set_small_penalty(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the SmallPenalty node.");
          result.successful = false;
        } else {
          camera_parameters_.small_penalty = new_value;
          RCLCPP_INFO(this->get_logger(), "small_penalty updated: %d",
                      camera_parameters_.small_penalty);
        }
      } else if (name == "stereo.large_penalty") {
        int new_value = param.as_int();
        if (new_value <= camera_parameters_.small_penalty) {
          RCLCPP_WARN(this->get_logger(),
                      "large_penalty must be greater than small_penalty. "
                      "Clamping value.");
          new_value = camera_parameters_.small_penalty + 1;
        }
        if (!SpinStereo::set_large_penalty(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the LargePenalty node.");
          result.successful = false;
        } else {
          camera_parameters_.large_penalty = new_value;
          RCLCPP_INFO(this->get_logger(), "large_penalty updated: %d",
                      camera_parameters_.large_penalty);
        }
      } else if (name == "stereo.uniqueness_ratio") {
        int new_value = param.as_int();
        if (!SpinStereo::set_uniqueness_ratio(p_cam_, new_value)) {
          RCLCPP_ERROR(this->get_logger(),
                       "Failed to set the UniquenessRatio node.");
          result.successful = false;
        } else {
          camera_parameters_.uniqueness_ratio = new_value;
          RCLCPP_INFO(this->get_logger(), "uniqueness_ratio updated: %d",
                      camera_parameters_.uniqueness_ratio);
        }
      } else if (name == "post_processing.enable") {
        camera_parameters_.post_process_disparity = param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "Post Processing Disparity: %s",
            camera_parameters_.post_process_disparity ? "enabled" : "disabled");
      } else if (name == "post_processing.max_speckle_size") {
        camera_parameters_.max_speckle_size = param.as_int();
        RCLCPP_INFO(this->get_logger(), "max_speckle_size updated: %d",
                    camera_parameters_.max_speckle_size);
      } else if (name == "post_processing.max_diff") {
        camera_parameters_.max_diff = param.as_double();
        RCLCPP_INFO(this->get_logger(), "max_diff updated: %f",
                    camera_parameters_.max_diff);
      } else if (name == "stream.raw_left_enabled") {
        camera_parameters_.stream_transmit_flags.raw_sensor_1_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(this->get_logger(), "raw_left_enabled updated: %s",
                    camera_parameters_.stream_transmit_flags
                            .raw_sensor_1_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (name == "stream.raw_right_enabled") {
        camera_parameters_.stream_transmit_flags.raw_sensor_2_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(this->get_logger(), "raw_right_enabled updated: %s",
                    camera_parameters_.stream_transmit_flags
                            .raw_sensor_2_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (name == "stream.rect_left_enabled") {
        camera_parameters_.stream_transmit_flags
            .rect_sensor_1_transmit_enabled = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rect_left_enabled updated: %s",
                    camera_parameters_.stream_transmit_flags
                            .rect_sensor_1_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (name == "stream.rect_right_enabled") {
        camera_parameters_.stream_transmit_flags
            .rect_sensor_2_transmit_enabled = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "rect_right_enabled updated: %s",
                    camera_parameters_.stream_transmit_flags
                            .rect_sensor_2_transmit_enabled
                        ? "enabled"
                        : "disabled");
        updatePublisher = true;
      } else if (name == "stream.disparity_enabled") {
        camera_parameters_.stream_transmit_flags.disparity_transmit_enabled =
            param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "disparity_transmit_enabled updated: %s",
            camera_parameters_.stream_transmit_flags.disparity_transmit_enabled
                ? "enabled"
                : "disabled");
        updatePublisher = true;
        if (!camera_parameters_.stream_transmit_flags
                 .disparity_transmit_enabled) {
          if (camera_parameters_.do_compute_point_cloud) {
            RCLCPP_WARN(this->get_logger(),
                        "Point cloud computation requires disparity "
                        "transmission to be enabled.");
          }
        }
      } else if (name == "camera.acquisition_frame_rate_enabled") {
        bool new_value = param.as_bool();
        if (new_value != camera_parameters_.acquisition_frame_rate_enabled) {
          camera_parameters_.acquisition_frame_rate_enabled = new_value;
          if (!SpinStereo::set_acquisition_frame_rate_enabled(
                  p_cam_, camera_parameters_.acquisition_frame_rate_enabled)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set the AcquisitionFrameRateEnable node.");
            result.successful = false;
          } else {
            RCLCPP_INFO(this->get_logger(),
                        "acquisition_frame_rate_enabled updated: %s",
                        camera_parameters_.acquisition_frame_rate_enabled
                            ? "enabled"
                            : "disabled");
          }
        }
      } else if (name == "camera.frame_rate") {
        double new_frame_rate_double = param.as_double();
        float new_frame_rate = static_cast<float>(new_frame_rate_double);
        if (std::abs(new_frame_rate - camera_parameters_.frame_rate) > 1e-6) {
          if (!SpinStereo::set_frame_rate(p_cam_, new_frame_rate)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the frame rate.");
            result.successful = false;
          } else {
            float updated_frame_rate;
            if (SpinStereo::get_frame_rate(p_cam_, updated_frame_rate)) {
              camera_parameters_.frame_rate = updated_frame_rate;
              RCLCPP_INFO(this->get_logger(), "frame_rate updated: %f",
                          camera_parameters_.frame_rate);
              // Reset timer with new frame rate
              timer_->cancel();
              timer_ = this->create_wall_timer(
                  std::chrono::milliseconds(
                      static_cast<int>(1000.0 / camera_parameters_.frame_rate)),
                  std::bind(&StereoImagePublisherNode::timer_callback, this));
            }
          }
        }
      } else if (name == "camera.auto_exposure") {
        bool new_value = param.as_bool();
        if (new_value != camera_parameters_.auto_exposure) {
          camera_parameters_.auto_exposure = new_value;
          if (new_value) {
            if (!SpinStereo::enable_auto_exposure(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to enable auto exposure.");
              result.successful = false;
            }
          } else {
            if (!SpinStereo::disable_auto_exposure(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(),
                           "Failed to disable auto exposure.");
              result.successful = false;
            }
          }
          RCLCPP_INFO(
              this->get_logger(), "auto_exposure updated: %s",
              camera_parameters_.auto_exposure ? "enabled" : "disabled");
        }
      } else if (name == "camera.exposure_time") {
        double new_exposure_time_double = param.as_double();
        float new_exposure_time = static_cast<float>(new_exposure_time_double);
        if (std::abs(new_exposure_time - camera_parameters_.exposure_time) >
            1e-6) {
          if (!SpinStereo::set_exposure_time(p_cam_, new_exposure_time)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to set the exposure time.");
            result.successful = false;
          } else {
            camera_parameters_.exposure_time = new_exposure_time;
            RCLCPP_INFO(this->get_logger(), "exposure_time updated: %f",
                        camera_parameters_.exposure_time);
          }
        }
      } else if (name == "camera.auto_gain") {
        bool new_value = param.as_bool();
        if (new_value != camera_parameters_.auto_gain) {
          camera_parameters_.auto_gain = new_value;
          if (new_value) {
            if (!SpinStereo::enable_auto_gain(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(), "Failed to enable auto gain.");
              result.successful = false;
            }
          } else {
            if (!SpinStereo::disable_auto_gain(p_cam_)) {
              RCLCPP_ERROR(this->get_logger(), "Failed to disable auto gain.");
              result.successful = false;
            }
          }
          RCLCPP_INFO(this->get_logger(), "auto_gain updated: %s",
                      camera_parameters_.auto_gain ? "enabled" : "disabled");
        }
      } else if (name == "camera.gain_value") {
        double new_gain_value_double = param.as_double();
        float new_gain_value = static_cast<float>(new_gain_value_double);
        if (std::abs(new_gain_value - camera_parameters_.gain_value) > 1e-6) {
          if (!SpinStereo::set_gain_value(p_cam_, new_gain_value)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set the gain value.");
            result.successful = false;
          } else {
            camera_parameters_.gain_value = new_gain_value;
            RCLCPP_INFO(this->get_logger(), "gain_value updated: %f",
                        camera_parameters_.gain_value);
          }
        }
      } else if (name == "camera.pixel_format") {
        std::string requested_format = param.as_string();
        if (requested_format != camera_parameters_.pixel_format) {
          if (!SpinStereo::set_pixel_format(p_cam_, requested_format)) {
            RCLCPP_ERROR(this->get_logger(),
                        "Failed to set pixel format to '%s'",
                        requested_format.c_str());
            result.successful = false;
          } else {
            camera_parameters_.pixel_format = requested_format;
            RCLCPP_INFO(this->get_logger(), "pixel_format updated: %s",
                        camera_parameters_.pixel_format.c_str());
            do_update_pixel_format_ = true;
            result.successful = true;
          }
        }
      } else if (name == "camera.resolution") {
        std::string requested = param.as_string();
        if (requested != camera_parameters_.resolution) {
          if (!SpinStereo::set_stereo_resolution(p_cam_, requested)) {
            RCLCPP_ERROR(this->get_logger(),
                        "Failed to set resolution to '%s'",
                        requested.c_str());
            result.successful = false;
          } else {
            camera_parameters_.resolution = requested;
            RCLCPP_INFO(this->get_logger(),
                        "resolution updated: %s",
                        camera_parameters_.resolution.c_str());
            do_update_resolution_ = true;
            result.successful = true;
          }
        }
      } else if (name == "point_cloud.enable") {
        camera_parameters_.do_compute_point_cloud = param.as_bool();
        RCLCPP_INFO(
            this->get_logger(), "Point cloud computation: %s",
            camera_parameters_.do_compute_point_cloud ? "enabled" : "disabled");
        updatePublisher = true;
        if (camera_parameters_.do_compute_point_cloud) {
          if (!camera_parameters_.stream_transmit_flags
                   .disparity_transmit_enabled) {
            RCLCPP_WARN(this->get_logger(),
                        "Point cloud computation requires disparity "
                        "transmission to be enabled.");
          }
        }
      } else if (name == "point_cloud.decimation_factor") {
        int new_value = param.as_int();
        new_value = std::clamp(new_value, 1, 20);
        point_cloud_parameters_.decimationFactor = new_value;
        RCLCPP_INFO(this->get_logger(), "decimation_factor updated: %d",
                    point_cloud_parameters_.decimationFactor);
      } else if (name.find("point_cloud.") == 0) {
        // Handle ROI parameters
        if (name == "point_cloud.XMin") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesXMin = new_value;
          RCLCPP_INFO(this->get_logger(), "XMin updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesXMin);
        } else if (name == "point_cloud.XMax") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesXMax = new_value;
          RCLCPP_INFO(this->get_logger(), "XMax updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesXMax);
        } else if (name == "point_cloud.YMin") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesYMin = new_value;
          RCLCPP_INFO(this->get_logger(), "YMin updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesYMin);
        } else if (name == "point_cloud.YMax") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesYMax = new_value;
          RCLCPP_INFO(this->get_logger(), "YMax updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesYMax);
        } else if (name == "point_cloud.ZMin") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesZMin = new_value;
          RCLCPP_INFO(this->get_logger(), "ZMin updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesZMin);
        } else if (name == "point_cloud.ZMax") {
          float new_value = static_cast<float>(param.as_double());
          point_cloud_parameters_.ROIWorldCoordinatesZMax = new_value;
          RCLCPP_INFO(this->get_logger(), "ZMax updated: %f",
                      point_cloud_parameters_.ROIWorldCoordinatesZMax);
        } else if (name == "point_cloud.ROI_image_left") {
          int new_value = param.as_int();
          int upper_limit;
          if (!this->get_parameter("point_cloud.ROI_image_right", upper_limit)) {
            int64_t stereo_width;
            if (!get_int_value_from_node(*node_map_, "StereoWidth", stereo_width)) {
              stereo_width = 2048;
            }
            upper_limit = (int)stereo_width;
          }
          new_value = std::clamp(new_value, 0, (int)upper_limit);
          point_cloud_parameters_.ROIImageLeft = new_value;
          RCLCPP_INFO(this->get_logger(), "ROI_image_left updated: %d",
                      point_cloud_parameters_.ROIImageLeft);
        } else if (name == "point_cloud.ROI_image_top") {
          int new_value = param.as_int();
          int upper_limit;
          if (!this->get_parameter("point_cloud.ROI_image_bottom", upper_limit)) {
            int64_t stereo_height;
            if (!get_int_value_from_node(*node_map_, "StereoHeight", stereo_height)) {
              stereo_height = 1536;
            }
            upper_limit = (int)stereo_height;
          }
          new_value = std::clamp(new_value, 0, (int)upper_limit);
          point_cloud_parameters_.ROIImageTop = new_value;
          RCLCPP_INFO(this->get_logger(), "ROI_image_top updated: %d",
                      point_cloud_parameters_.ROIImageTop);
        } else if (name == "point_cloud.ROI_image_right") {
          int new_value = param.as_int();
          int lower_limit;
          if (!this->get_parameter("point_cloud.ROI_image_left", lower_limit)) {
            lower_limit = 0;
          }
          int64_t stereo_width;
          if (!get_int_value_from_node(*node_map_, "StereoWidth", stereo_width)) {
            stereo_width = 2048;
          }
          new_value = std::clamp(new_value, lower_limit, (int)stereo_width);
          point_cloud_parameters_.ROIImageRight = new_value;
          RCLCPP_INFO(this->get_logger(), "ROI_image_right updated: %d",
                      point_cloud_parameters_.ROIImageRight);
        } else if (name == "point_cloud.ROI_image_bottom") {
          int new_value = param.as_int();
          int lower_limit;
          if (!this->get_parameter("point_cloud.ROI_image_top", lower_limit)) {
            lower_limit = 0;
          }
          int64_t stereo_height;
          if (!get_int_value_from_node(*node_map_, "StereoHeight", stereo_height)) {
            stereo_height = 1536;
          }
          new_value = std::clamp(new_value, lower_limit, (int)stereo_height);
          point_cloud_parameters_.ROIImageBottom = new_value;
          RCLCPP_INFO(this->get_logger(), "ROI_image_botom updated: %d",
                      point_cloud_parameters_.ROIImageBottom);
        }
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
            camera_parameters_.stream_transmit_flags
                .raw_sensor_1_transmit_enabled ||
            camera_parameters_.stream_transmit_flags
                .raw_sensor_2_transmit_enabled ||
            camera_parameters_.stream_transmit_flags
                .rect_sensor_1_transmit_enabled ||
            camera_parameters_.stream_transmit_flags
                .rect_sensor_2_transmit_enabled ||
            camera_parameters_.stream_transmit_flags.disparity_transmit_enabled;

        if (at_least_one_enabled) {
          // Set throughput to max
          RCLCPP_DEBUG(
              this->get_logger(),
              "Trying to set DeviceLinkThroughputLimit to DeviceMaxThroughput");
          if (!set_device_link_throughput_to_max(p_cam_)) {
            RCLCPP_WARN(this->get_logger(),
                        "Unable to set DeviceLinkThroughputLimit to "
                        "DeviceMaxThroughput");
          }

          // Reconfigure the camera streams
          if (!SpinStereo::configure_camera_streams(
                  p_cam_, camera_parameters_.stream_transmit_flags)) {
            RCLCPP_ERROR(this->get_logger(),
                         "Error during stream reconfiguration: %s",
                         "Could not configure camera streams.");
            result.successful = false;
            return result;
          }

          // Set throughput to current
          RCLCPP_DEBUG(this->get_logger(),
                       "Trying to set DeviceLinkThroughputLimit to "
                       "DeviceLinkCurrentThroughput");
          if (!set_device_link_throughput_to_current(p_cam_)) {
            RCLCPP_WARN(this->get_logger(),
                        "Unable to set DeviceLinkThroughputLimit to "
                        "DeviceLinkCurrentThroughput. "
                        "This might cause packet loss.");
          }

          // if more streams are enabled, the frame rate might change. Updating
          // the resulting frame rate accordingly.
          if (!get_frame_rate(p_cam_, camera_parameters_.frame_rate)) {
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

        initialize_publishers();

      } catch (const Spinnaker::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "Error during stream reconfiguration: %s", e.what());
        result.successful = false;
      }
    }

    return result;
  }

  void poll_node_map_parameters() {
    // Poll min_disparity (Scan3dCoordinateOffset)
    float scan3d_coordinate_offset;
    if (get_float_value_from_node(*node_map_, "Scan3dCoordinateOffset",
                                  scan3d_coordinate_offset)) {
      // First if: Update camera_parameters_ if value changed
      if (std::abs(camera_parameters_.scan3d_coordinate_offset -
                   scan3d_coordinate_offset) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters_.scan3d_coordinate_offset = scan3d_coordinate_offset;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_min_disp;
    if (this->get_parameter("stereo.min_disparity",
                            current_ros_param_min_disp)) {
      if (std::abs(camera_parameters_.scan3d_coordinate_offset -
                   static_cast<float>(current_ros_param_min_disp)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.min_disparity",
            static_cast<double>(camera_parameters_.scan3d_coordinate_offset)));
        RCLCPP_DEBUG(this->get_logger(),
                     "min_disparity (Scan3dCoordinateOffset) updated from "
                     "camera node: %f",
                     camera_parameters_.scan3d_coordinate_offset);
      }
    }

    // Poll small_penalty
    int64_t small_penalty;
    if (get_int_value_from_node(*node_map_, "SmallPenalty", small_penalty)) {
      // First if: Update camera_parameters_ if value changed
      if (camera_parameters_.small_penalty != small_penalty) {
        camera_parameters_.small_penalty = small_penalty;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_small_penalty;
    if (this->get_parameter("stereo.small_penalty",
                            current_ros_param_small_penalty)) {
      if (camera_parameters_.small_penalty != current_ros_param_small_penalty) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.small_penalty",
            static_cast<int>(camera_parameters_.small_penalty)));
        RCLCPP_DEBUG(this->get_logger(),
                     "small_penalty updated from camera node: %d",
                     camera_parameters_.small_penalty);
      }
    }

    // Poll large_penalty
    int64_t large_penalty;
    if (get_int_value_from_node(*node_map_, "LargePenalty", large_penalty)) {
      // First if: Update camera_parameters_ if value changed
      if (camera_parameters_.large_penalty != large_penalty) {
        camera_parameters_.large_penalty = large_penalty;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_large_penalty;
    if (this->get_parameter("stereo.large_penalty",
                            current_ros_param_large_penalty)) {
      if (camera_parameters_.large_penalty != current_ros_param_large_penalty) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.large_penalty",
            static_cast<int>(camera_parameters_.large_penalty)));
        RCLCPP_DEBUG(this->get_logger(),
                     "large_penalty updated from camera node: %d",
                     camera_parameters_.large_penalty);
      }
    }

    // Poll uniqueness_ratio
    int64_t uniqueness_ratio;
    if (get_int_value_from_node(*node_map_, "UniquenessRatio",
                                uniqueness_ratio)) {
      // First if: Update camera_parameters_ if value changed
      if (camera_parameters_.uniqueness_ratio != uniqueness_ratio) {
        camera_parameters_.uniqueness_ratio = uniqueness_ratio;
      }
    }

    // Second if: Update ROS parameter if value changed
    int current_ros_param_uniqueness_ratio;
    if (this->get_parameter("stereo.uniqueness_ratio",
                            current_ros_param_uniqueness_ratio)) {
      if (camera_parameters_.uniqueness_ratio !=
          current_ros_param_uniqueness_ratio) {
        this->set_parameter(rclcpp::Parameter(
            "stereo.uniqueness_ratio",
            static_cast<int>(camera_parameters_.uniqueness_ratio)));
        RCLCPP_DEBUG(this->get_logger(),
                     "uniqueness_ratio updated from camera node: %d",
                     camera_parameters_.uniqueness_ratio);
      }
    }

    // Poll acquisition frame rate enabled status
    bool acquisition_frame_rate_enabled =
        is_acquisition_frame_rate_enabled(p_cam_);
    if (camera_parameters_.acquisition_frame_rate_enabled !=
        acquisition_frame_rate_enabled) {
      camera_parameters_.acquisition_frame_rate_enabled =
          acquisition_frame_rate_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_acquisition_frame_rate_enabled;
    if (this->get_parameter("camera.acquisition_frame_rate_enabled",
                            current_ros_param_acquisition_frame_rate_enabled)) {
      if (camera_parameters_.acquisition_frame_rate_enabled !=
          current_ros_param_acquisition_frame_rate_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "camera.acquisition_frame_rate_enabled",
            camera_parameters_.acquisition_frame_rate_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "acquisition_frame_rate_enabled updated from camera node: %s",
            camera_parameters_.acquisition_frame_rate_enabled ? "enabled"
                                                              : "disabled");
      }
    }

    // Poll frame_rate
    float frame_rate;
    if (get_frame_rate(p_cam_, frame_rate)) {
      if (std::abs(camera_parameters_.frame_rate - frame_rate) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters_.frame_rate = frame_rate;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_frame_rate;
    if (this->get_parameter("camera.frame_rate",
                            current_ros_param_frame_rate)) {
      if (std::abs(camera_parameters_.frame_rate -
                   static_cast<float>(current_ros_param_frame_rate)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.frame_rate",
            static_cast<double>(camera_parameters_.frame_rate)));
        RCLCPP_DEBUG(this->get_logger(),
                     "frame_rate updated from camera node: %f",
                     camera_parameters_.frame_rate);
      }
    }

    // Poll auto exposure enabled status
    bool auto_exposure = is_auto_exposure_enabled(p_cam_);
    if (camera_parameters_.auto_exposure != auto_exposure) {
      camera_parameters_.auto_exposure = auto_exposure;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_auto_exposure;
    if (this->get_parameter("camera.auto_exposure",
                            current_ros_param_auto_exposure)) {
      if (camera_parameters_.auto_exposure != current_ros_param_auto_exposure) {
        this->set_parameter(rclcpp::Parameter(
            "camera.auto_exposure", camera_parameters_.auto_exposure));
        RCLCPP_DEBUG(this->get_logger(),
                     "auto_exposure updated from camera node: %s",
                     camera_parameters_.auto_exposure ? "enabled" : "disabled");
      }
    }

    // Poll exposure_time
    float exposure_time;
    if (get_exposure_time(p_cam_, exposure_time)) {
      if (std::abs(camera_parameters_.exposure_time - exposure_time) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters_.exposure_time = exposure_time;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_exposure_time;
    if (this->get_parameter("camera.exposure_time",
                            current_ros_param_exposure_time)) {
      if (std::abs(camera_parameters_.exposure_time -
                   static_cast<float>(current_ros_param_exposure_time)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.exposure_time",
            static_cast<double>(camera_parameters_.exposure_time)));
        RCLCPP_DEBUG(this->get_logger(),
                     "exposure_time updated from camera node: %f",
                     camera_parameters_.exposure_time);
      }
    }

    // Poll auto gain enabled status
    bool auto_gain = is_auto_gain_enabled(p_cam_);
    if (camera_parameters_.auto_gain != auto_gain) {
      camera_parameters_.auto_gain = auto_gain;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_auto_gain;
    if (this->get_parameter("camera.auto_gain", current_ros_param_auto_gain)) {
      if (camera_parameters_.auto_gain != current_ros_param_auto_gain) {
        this->set_parameter(rclcpp::Parameter("camera.auto_gain",
                                              camera_parameters_.auto_gain));
        RCLCPP_DEBUG(this->get_logger(),
                     "auto_gain updated from camera node: %s",
                     camera_parameters_.auto_gain ? "enabled" : "disabled");
      }
    }

    // Poll gain_value
    float gain_value;
    if (get_gain_value(p_cam_, gain_value)) {
      if (std::abs(camera_parameters_.gain_value - gain_value) >
          std::numeric_limits<float>::epsilon()) {
        camera_parameters_.gain_value = gain_value;
      }
    }

    // Second if: Update ROS parameter if value changed
    double current_ros_param_gain_value;
    if (this->get_parameter("camera.gain_value",
                            current_ros_param_gain_value)) {
      if (std::abs(camera_parameters_.gain_value -
                   static_cast<float>(current_ros_param_gain_value)) >
          std::numeric_limits<float>::epsilon()) {
        this->set_parameter(rclcpp::Parameter(
            "camera.gain_value",
            static_cast<double>(camera_parameters_.gain_value)));
        RCLCPP_DEBUG(this->get_logger(),
                     "gain_value updated from camera node: %f",
                     camera_parameters_.gain_value);
      }
    }

    // Poll pixel_format
    std::string polled_format;
    if (get_pixel_format(p_cam_, polled_format)) {
      if (camera_parameters_.pixel_format != polled_format) {
        camera_parameters_.pixel_format = polled_format;
      }
    }

    // Second if: Update ROS parameter if value changed
    std::string current_ros_param_format;
    if (this->get_parameter("camera.pixel_format", current_ros_param_format)) {
      if (camera_parameters_.pixel_format != current_ros_param_format || do_update_pixel_format_) {
        this->set_parameter(rclcpp::Parameter("camera.pixel_format", camera_parameters_.pixel_format));
        RCLCPP_DEBUG(this->get_logger(),
                    "pixel_format updated from camera node: %s",
                    camera_parameters_.pixel_format.c_str());
        do_update_pixel_format_ = false;
      }
    }

    // --- Poll stereo_resolution
    std::string polled_resolution;
    if (get_stereo_resolution(p_cam_, polled_resolution)) {
      if (camera_parameters_.resolution != polled_resolution) {
        camera_parameters_.resolution = polled_resolution;
      }
    }

    // Second if: Update ROS parameter if value changed (or we flagged a fallback)
    std::string current_ros_param_resolution;
    if (this->get_parameter("camera.resolution", current_ros_param_resolution)) {
      if (camera_parameters_.resolution != current_ros_param_resolution
          || do_update_resolution_)
      {
        this->set_parameter(
          rclcpp::Parameter("camera.resolution",
                            camera_parameters_.resolution));
        RCLCPP_DEBUG(this->get_logger(),
                    "camera.resolution updated from camera node: %s",
                    camera_parameters_.resolution.c_str());

        if (!configure_stereo_processing(p_cam_->GetNodeMap(),
                                        stereo_parameters_)) {
          RCLCPP_ERROR(this->get_logger(),
                      "Failed to configure stereo processing parameters after updating stereo resolution.");
        } else {
          this->set_parameter(rclcpp::Parameter("stereo.focal_length", stereo_parameters_.focalLength));
          this->set_parameter(rclcpp::Parameter("stereo.principal_point_u", stereo_parameters_.principalPointU));
          this->set_parameter(rclcpp::Parameter("stereo.principal_point_v", stereo_parameters_.principalPointV));
          
          point_cloud_generator.setPointCloudExtents(
            p_cam_->GetNodeMap(), point_cloud_parameters_);

          // and update the ROS parameters so your clamping ranges also refresh
          this->set_parameter(
            rclcpp::Parameter("point_cloud.ROI_image_left",
                              static_cast<int64_t>(point_cloud_parameters_.ROIImageLeft)));
          this->set_parameter(
            rclcpp::Parameter("point_cloud.ROI_image_top",
                              static_cast<int64_t>(point_cloud_parameters_.ROIImageTop)));
          this->set_parameter(
            rclcpp::Parameter("point_cloud.ROI_image_right",
                              static_cast<int64_t>(point_cloud_parameters_.ROIImageRight)));
          this->set_parameter(
            rclcpp::Parameter("point_cloud.ROI_image_bottom",
                              static_cast<int64_t>(point_cloud_parameters_.ROIImageBottom)));
        }
        do_update_resolution_ = false;
      }
    }

    int current_ros_param_ROI;
    if (this->get_parameter("point_cloud.ROI_image_left", current_ros_param_ROI)) {
      if (point_cloud_parameters_.ROIImageLeft != current_ros_param_ROI) {
        this->set_parameter(
          rclcpp::Parameter("point_cloud.ROI_image_left", static_cast<int64_t>(point_cloud_parameters_.ROIImageLeft)));
      }
    }
    if (this->get_parameter("point_cloud.ROI_image_right", current_ros_param_ROI)) {
      if (point_cloud_parameters_.ROIImageRight != current_ros_param_ROI) {
        this->set_parameter(
          rclcpp::Parameter("point_cloud.ROI_image_right", static_cast<int64_t>(point_cloud_parameters_.ROIImageRight)));
      }
    }
    if (this->get_parameter("point_cloud.ROI_image_top", current_ros_param_ROI)) {
      if (point_cloud_parameters_.ROIImageTop != current_ros_param_ROI) {
        this->set_parameter(
          rclcpp::Parameter("point_cloud.ROI_image_top", static_cast<int64_t>(point_cloud_parameters_.ROIImageTop)));
      }
    }
    if (this->get_parameter("point_cloud.ROI_image_bottom", current_ros_param_ROI)) {
      if (point_cloud_parameters_.ROIImageBottom != current_ros_param_ROI) {
        this->set_parameter(
          rclcpp::Parameter("point_cloud.ROI_image_bottom", static_cast<int64_t>(point_cloud_parameters_.ROIImageBottom)));
      }
    }

    // Poll raw_sensor_1_transmit_enabled
    bool raw_sensor1_transmit_enabled =
        is_raw_sensor_1_transmit_enabled(p_cam_);
    if (camera_parameters_.stream_transmit_flags
            .raw_sensor_1_transmit_enabled != raw_sensor1_transmit_enabled) {
      camera_parameters_.stream_transmit_flags.raw_sensor_1_transmit_enabled =
          raw_sensor1_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_raw_sensor1_transmit_enabled;
    if (this->get_parameter("stream.raw_left_enabled",
                            current_ros_param_raw_sensor1_transmit_enabled)) {
      if (camera_parameters_.stream_transmit_flags
              .raw_sensor_1_transmit_enabled !=
          current_ros_param_raw_sensor1_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.raw_left_enabled", camera_parameters_.stream_transmit_flags
                                           .raw_sensor_1_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "raw_sensor_1_transmit_enabled updated from camera node: %s",
            camera_parameters_.stream_transmit_flags
                    .raw_sensor_1_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll raw_sensor_2_transmit_enabled
    bool raw_sensor2_transmit_enabled =
        is_raw_sensor_2_transmit_enabled(p_cam_);
    if (camera_parameters_.stream_transmit_flags
            .raw_sensor_2_transmit_enabled != raw_sensor2_transmit_enabled) {
      camera_parameters_.stream_transmit_flags.raw_sensor_2_transmit_enabled =
          raw_sensor2_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_raw_sensor2_transmit_enabled;
    if (this->get_parameter("stream.raw_right_enabled",
                            current_ros_param_raw_sensor2_transmit_enabled)) {
      if (camera_parameters_.stream_transmit_flags
              .raw_sensor_2_transmit_enabled !=
          current_ros_param_raw_sensor2_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.raw_right_enabled", camera_parameters_.stream_transmit_flags
                                            .raw_sensor_2_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "raw_sensor_2_transmit_enabled updated from camera node: %s",
            camera_parameters_.stream_transmit_flags
                    .raw_sensor_2_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll rect_sensor_1_transmit_enabled
    bool rect_sensor1_transmit_enabled =
        is_rectified_sensor_1_transmit_enabled(p_cam_);
    if (camera_parameters_.stream_transmit_flags
            .rect_sensor_1_transmit_enabled != rect_sensor1_transmit_enabled) {
      camera_parameters_.stream_transmit_flags.rect_sensor_1_transmit_enabled =
          rect_sensor1_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_rect_sensor1_transmit_enabled;
    if (this->get_parameter("stream.rect_left_enabled",
                            current_ros_param_rect_sensor1_transmit_enabled)) {
      if (camera_parameters_.stream_transmit_flags
              .rect_sensor_1_transmit_enabled !=
          current_ros_param_rect_sensor1_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.rect_left_enabled", camera_parameters_.stream_transmit_flags
                                            .rect_sensor_1_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "rect_sensor_1_transmit_enabled updated from camera node: %s",
            camera_parameters_.stream_transmit_flags
                    .rect_sensor_1_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll rect_sensor_2_transmit_enabled
    bool rect_sensor2_transmit_enabled =
        is_rectified_sensor_2_transmit_enabled(p_cam_);
    if (camera_parameters_.stream_transmit_flags
            .rect_sensor_2_transmit_enabled != rect_sensor2_transmit_enabled) {
      camera_parameters_.stream_transmit_flags.rect_sensor_2_transmit_enabled =
          rect_sensor2_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_rect_sensor2_transmit_enabled;
    if (this->get_parameter("stream.rect_right_enabled",
                            current_ros_param_rect_sensor2_transmit_enabled)) {
      if (camera_parameters_.stream_transmit_flags
              .rect_sensor_2_transmit_enabled !=
          current_ros_param_rect_sensor2_transmit_enabled) {
        this->set_parameter(
            rclcpp::Parameter("stream.rect_right_enabled",
                              camera_parameters_.stream_transmit_flags
                                  .rect_sensor_2_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "rect_sensor_2_transmit_enabled updated from camera node: %s",
            camera_parameters_.stream_transmit_flags
                    .rect_sensor_2_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }

    // Poll disparity_transmit_enabled
    bool disparity_transmit_enabled = is_disparity_transmit_enabled(p_cam_);
    if (camera_parameters_.stream_transmit_flags.disparity_transmit_enabled !=
        disparity_transmit_enabled) {
      camera_parameters_.stream_transmit_flags.disparity_transmit_enabled =
          disparity_transmit_enabled;
    }

    // Second if: Update ROS parameter if value changed
    bool current_ros_param_disparity_transmit_enabled;
    if (this->get_parameter("stream.disparity_enabled",
                            current_ros_param_disparity_transmit_enabled)) {
      if (camera_parameters_.stream_transmit_flags.disparity_transmit_enabled !=
          current_ros_param_disparity_transmit_enabled) {
        this->set_parameter(rclcpp::Parameter(
            "stream.disparity_enabled", camera_parameters_.stream_transmit_flags
                                            .disparity_transmit_enabled));
        RCLCPP_DEBUG(
            this->get_logger(),
            "disparity_transmit_enabled updated from camera node: %s",
            camera_parameters_.stream_transmit_flags.disparity_transmit_enabled
                ? "enabled"
                : "disabled");
      }
    }
  }

  // Timer callback for main loop
  void timer_callback() {
    // Acquire the mutex at the beginning of the callback
    std::lock_guard<std::mutex> lock(camera_mutex_);

    ImageList image_list;
    ImagePtr disparity_image;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    int count = 1;
    bool save =
        false;  // Set to true if you want to save images and point clouds

    if (p_cam_->IsStreaming()) {
      // Capture the next set of images from the camera.
      uint64_t timeout_milli_secs =
          static_cast<uint64_t>((1000.0 / camera_parameters_.frame_rate) * 2.0);
      if (!SpinStereo::get_next_image_set(
              p_cam_, camera_parameters_.stream_transmit_flags,
              timeout_milli_secs, image_list)) {
        RCLCPP_WARN(this->get_logger(), "Failed to get next image set.");
        return;
      }

      // Apply post-processing on the disparity image if the corresponding flags
      // are enabled. Ensure the disparity image is not incomplete before
      // applying the filter.
      if (camera_parameters_.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {
        if (camera_parameters_.post_process_disparity) {
          // Apply a speckle filter to clean up the disparity image using the
          // provided parameters.
          disparity_image = ImageUtilityStereo::FilterSpeckles(
              image_list.GetByPayloadType(
                  SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1),
              camera_parameters_.max_speckle_size, camera_parameters_.max_diff,
              stereo_parameters_.disparityScaleFactor,
              stereo_parameters_.invalidDataValue);
        } else {
          // If no post-processing is needed, simply use the disparity image as
          // it is.
          disparity_image = image_list.GetByPayloadType(
              SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1);
        }
      }

      // If point cloud computation is enabled and the disparity image is valid:
      if (camera_parameters_.do_compute_point_cloud &&
          camera_parameters_.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {

        point_cloud_generator.compute_point_cloud(
              disparity_image,
              image_list.GetByPayloadType(
                  SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1),
                  stereo_parameters_, point_cloud_parameters_,
                  point_cloud);

        // // Clear the existing point cloud to prepare for new data.
        // point_cloud->clear();

        // float min_disparity = camera_parameters_.scan3d_coordinate_offset;

        // // If the rectified image is available, use it to color the point cloud.
        // if (camera_parameters_.stream_transmit_flags
        //         .rect_sensor_1_transmit_enabled &&
        //     !image_list
        //          .GetByPayloadType(
        //              SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1)
        //          ->IsIncomplete()) {
        //   point_cloud_generator.compute_point_cloud(
        //       disparity_image,
        //       image_list.GetByPayloadType(
        //           SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1),
        //       min_disparity, stereo_parameters_, point_cloud_parameters_,
        //       point_cloud);
        // } else {
        //   // If no rectified image is available, compute the point cloud without
        //   // coloring.
        //   point_cloud_generator.compute_point_cloud(
        //       disparity_image, min_disparity, stereo_parameters_,
        //       point_cloud_parameters_, point_cloud);
        // }
      }

      // Get current ROS2 timestamp
      auto time_stamp = this->now();

      // Publish the raw left image if enabled and the image is complete.
      if (camera_parameters_.stream_transmit_flags
              .raw_sensor_1_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1)
               ->IsIncomplete()) {
        this->publish_image(image_list.GetByPayloadType(
                                SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1),
                            raw_left_image_publisher_, time_stamp);
        if (save) {
          std::ostringstream filename;
          filename << "rawLeft_" << count << ".png";
          image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR1)
              ->Save(filename.str().c_str());
        }
      }

      // Publish the raw right image if enabled and the image is complete.
      if (camera_parameters_.stream_transmit_flags
              .raw_sensor_2_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2)
               ->IsIncomplete()) {
        this->publish_image(image_list.GetByPayloadType(
                                SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2),
                            raw_right_image_publisher_, time_stamp);
        if (save) {
          std::ostringstream filename;
          filename << "rawRight_" << count << ".png";
          image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RAW_SENSOR2)
              ->Save(filename.str().c_str());
        }
      }

      // Publish the rectified left image if enabled and the image is complete.
      if (camera_parameters_.stream_transmit_flags
              .rect_sensor_1_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1)
               ->IsIncomplete() &&
          rect_left_image_publisher_ &&
          rect_left_camera_info_publisher_) {
        this->publish_image(image_list.GetByPayloadType(
                                SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1),
                            rect_left_image_publisher_, time_stamp);
                                                     
        auto rect_left_img_ptr = image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1);
        int imageWidth = rect_left_img_ptr->GetWidth();
        int imageHeight = rect_left_img_ptr->GetHeight();                            
        sensor_msgs::msg::CameraInfo cam_info = generateCameraInfo(
                                                stereo_parameters_,
                                                imageWidth,
                                                imageHeight);
        cam_info.header.stamp = time_stamp;
        cam_info.header.frame_id = "camera_left_optical_frame";
        rect_left_camera_info_publisher_->publish(cam_info);
        
        if (save) {
          std::ostringstream filename;
          filename << "rectLeft_" << count << ".png";
          image_list
              .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR1)
              ->Save(filename.str().c_str());
        }
      }

      // Publish the rectified right image if enabled and the image is complete.
      if (camera_parameters_.stream_transmit_flags
              .rect_sensor_2_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2)
               ->IsIncomplete() &&
          rect_right_image_publisher_ &&
          rect_right_camera_info_publisher_) {
        this->publish_image(image_list.GetByPayloadType(
                                SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2),
                            rect_right_image_publisher_, time_stamp);
                                                     
        auto rect_right_img_ptr = image_list.GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2);
        int imageWidth = rect_right_img_ptr->GetWidth();
        int imageHeight = rect_right_img_ptr->GetHeight();
        sensor_msgs::msg::CameraInfo cam_info = generateCameraInfo(
                                                stereo_parameters_,
                                                imageWidth,
                                                imageHeight);
        cam_info.header.stamp = time_stamp;
        cam_info.header.frame_id = "camera_right_optical_frame";
        rect_right_camera_info_publisher_->publish(cam_info);
        
        if (save) {
          std::ostringstream filename;
          filename << "rectRight_" << count << ".png";
          image_list
              .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_RECTIFIED_SENSOR2)
              ->Save(filename.str().c_str());
        }
      }

      // Publish the disparity image if enabled and the image is complete.
      if (camera_parameters_.stream_transmit_flags.disparity_transmit_enabled &&
          !image_list
               .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
               ->IsIncomplete()) {
        this->publish_disparity_image(disparity_image, time_stamp, save, count);
        if (save) {
          std::ostringstream filename;
          filename << "monoDisparity_" << count << ".png";
          image_list
              .GetByPayloadType(SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
              ->Save(filename.str().c_str());
        }
      }

      // Publish the point cloud if enabled and the disparity image is
      // available.
      if (camera_parameters_.do_compute_point_cloud) {
        if (camera_parameters_.stream_transmit_flags
                .disparity_transmit_enabled &&
            !image_list
                 .GetByPayloadType(
                     SPINNAKER_IMAGE_PAYLOAD_TYPE_DISPARITY_SENSOR1)
                 ->IsIncomplete()) {
          this->publish_point_cloud(point_cloud, time_stamp, save, count);
        }
      }

      // Increment count for potential saving
      count++;

      // Check if the camera is still valid
      if (!p_cam_->IsValid()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "The camera seems to be disconnected. Attempting to reconnect...");
        // Implement reconnection logic
        reconnect_camera();
      }
    }
  }

  // Reconnect camera logic
  void reconnect_camera() {
    // Acquire the mutex at the beginning
    std::lock_guard<std::mutex> lock(camera_mutex_);
    // Attempt to reconnect to the camera
    try {
      p_cam_->DeInit();
      p_cam_->Init();
      p_cam_->BeginAcquisition();
      RCLCPP_INFO(this->get_logger(), "Camera reconnected successfully.");
    } catch (const Spinnaker::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                   "Exception during camera reconnection: %s", e.what());
      // Optionally implement retry mechanism or notify user
    }
  }

  // Publish ROS Image message
  void publish_image(
      ImagePtr imagePtr,
      rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher,
      rclcpp::Time time_stamp) {

    if (!publisher || !imagePtr || !imagePtr->GetData()) {
      RCLCPP_WARN(this->get_logger(), "Publisher or image data is not initialized.");
      return;
    }

    try {
      cv::Mat cvImage;
      std::string encoding;
      auto pixelFormat = imagePtr->GetPixelFormat();
      Spinnaker::ImagePtr convertedImage = imagePtr;

      // Create an ImageProcessor to handle format conversions
      Spinnaker::ImageProcessor processor;

      switch (pixelFormat) {
        case PixelFormat_RGB8:
          cvImage = cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(),
                            CV_8UC3, imagePtr->GetData(), imagePtr->GetStride());
          encoding = "rgb8";
          break;

        case PixelFormat_RGB12p:
          convertedImage = processor.Convert(imagePtr, PixelFormat_RGB8);
          cvImage = cv::Mat(convertedImage->GetHeight(), convertedImage->GetWidth(),
                            CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
          encoding = "rgb8";
          break;

        case PixelFormat_Mono8:
          cvImage = cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(),
                            CV_8UC1, imagePtr->GetData(), imagePtr->GetStride());
          encoding = "mono8";
          break;

        case PixelFormat_Mono12Packed:
          convertedImage = processor.Convert(imagePtr, PixelFormat_Mono8);
          cvImage = cv::Mat(convertedImage->GetHeight(), convertedImage->GetWidth(),
                            CV_8UC1, convertedImage->GetData(), convertedImage->GetStride());
          encoding = "mono8";
          break;

        case PixelFormat_YUV422Packed:
          convertedImage = processor.Convert(imagePtr, PixelFormat_RGB8);
          cvImage = cv::Mat(convertedImage->GetHeight(), convertedImage->GetWidth(),
                            CV_8UC3, convertedImage->GetData(), convertedImage->GetStride());
          encoding = "rgb8";
          break;

        default:
          RCLCPP_ERROR(this->get_logger(), "Unsupported pixel format: %d",
                      static_cast<int>(pixelFormat));
          return;
      }

      auto msg =
          cv_bridge::CvImage(std_msgs::msg::Header(), encoding, cvImage).toImageMsg();
      msg->header.stamp = time_stamp;
      publisher->publish(*msg);

    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                  "OpenCV exception during image publishing: %s", e.what());
    } catch (const Spinnaker::Exception& e) {
      RCLCPP_ERROR(this->get_logger(),
                  "Spinnaker exception during image conversion: %s", e.what());
    }
  }


  // Publish ROS disparity image message
  void publish_disparity_image(ImagePtr disparity_image_ptr,
                               rclcpp::Time time_stamp, bool save, int count) {
    if (disparity_image_publisher_ && disparity_image_ptr->GetData()) {
      try {
        cv::Mat disparity_image(disparity_image_ptr->GetHeight(),
                                disparity_image_ptr->GetWidth(), CV_16UC1,
                                disparity_image_ptr->GetData(),
                                disparity_image_ptr->GetStride());

        // Convert disparity to depth visualization
        float min_depth =
            (stereo_parameters_.focalLength * stereo_parameters_.baseline) /
            (256.0f + camera_parameters_.scan3d_coordinate_offset);
        double max_depth = 20.0;  // Example maximum depth, adjust as needed

        cv::Mat depth_image = cv::Mat::zeros(disparity_image.size(), CV_32F);

        for (int row = 0; row < disparity_image.rows; ++row) {
          for (int col = 0; col < disparity_image.cols; ++col) {
            uint16_t d = disparity_image.at<uint16_t>(row, col);
            if (d == 0) {
              depth_image.at<float>(row, col) = 0.0f;
            } else {
              float disparity = static_cast<float>(d) *
                                    stereo_parameters_.disparityScaleFactor +
                                camera_parameters_.scan3d_coordinate_offset;
              float depth = (stereo_parameters_.focalLength *
                             stereo_parameters_.baseline) /
                            disparity;
              if (depth > max_depth || depth < min_depth) {
                depth_image.at<float>(row, col) = 0.0f;
              } else {
                depth_image.at<float>(row, col) = depth;
              }
            }
          }
        }

        cv::Mat normalized_depth;
        cv::normalize(depth_image, normalized_depth, 0, 1, cv::NORM_MINMAX);

        cv::Mat depth_8bits;
        normalized_depth.convertTo(depth_8bits, CV_8U, 255);

        cv::Mat colormap;
        cv::applyColorMap(depth_8bits, colormap, cv::COLORMAP_HSV);

        // Mask invalid depths
        cv::Mat mask = (depth_image == 0);
        colormap.setTo(cv::Scalar(0, 0, 0), mask);

        if (save) {
          std::ostringstream filename;
          filename << "colored_depth_" << count << ".png";
          cv::imwrite(filename.str(), colormap);
        }

        // Convert to ROS Image message
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colormap)
                       .toImageMsg();
        msg->header.stamp = time_stamp;
        disparity_image_publisher_->publish(*msg);
      } catch (const cv::Exception& e) {
        RCLCPP_ERROR(this->get_logger(),
                     "OpenCV exception during disparity image publishing: %s",
                     e.what());
      }
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Disparity image publisher or image data is not initialized.");
    }
  }

  // Publish ROS PointCloud2 message
  void publish_point_cloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr,
      rclcpp::Time time_stamp, bool save, int count) {
    if (point_cloud_publisher_) {
      sensor_msgs::msg::PointCloud2 msg;
      pcl::toROSMsg(*point_cloud_ptr, msg);
      msg.header.frame_id = "camera_link";
      msg.header.stamp = time_stamp;
      point_cloud_publisher_->publish(msg);

      if (save) {
        std::ostringstream filename;
        filename << "pointCloud_" << count << ".pcd";
        pcl::io::savePCDFileBinary(filename.str(), *point_cloud_ptr);
      }
    } else {
      RCLCPP_WARN(
          this->get_logger(),
          "Point cloud publisher is not initialized or point cloud is empty.");
    }
  }
};

int main(int argc, char** argv) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(true);

  // Create the node
  auto node = std::make_shared<StereoImagePublisherNode>(opts);

  // Spin the node with a MultiThreadedExecutor
  // rclcpp::executors::MultiThreadedExecutor executor;
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}
