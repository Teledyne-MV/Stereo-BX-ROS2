#define ROLLING_TESTING

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#ifdef HUMBLE_RELEASE
#include <cv_bridge/cv_bridge.h>
#else // HUMBLE_RELEASE
#endif // HUMBLE_RELEASE

#ifdef ROLLING_TESTING
#include <cv_bridge/cv_bridge.hpp>
#else // ROLLING_TESTING
#endif // ROLLING_TESTING

#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

#include "Spinnaker.h"
#include "stereo_image_publisher/SpinStereoHelper.h"
#include "stereo_image_publisher/StereoParameters.h"

using namespace Spinnaker;
using namespace std;
using namespace SpinStereo;

bool disablePacketResends(CameraPtr pCam)
{
    try
    {
        // Retrieve TLStream nodemap (Transport Layer Stream)
        INodeMap& nodeMapTLStream = pCam->GetTLStreamNodeMap();

        // Get the packet resend enable node
        CBooleanPtr ptrPacketResendEnable = nodeMapTLStream.GetNode("StreamPacketResendEnable");

        if (IsWritable(ptrPacketResendEnable))
        {
            ptrPacketResendEnable->SetValue(false);
            std::cout << "Packet resends disabled." << std::endl;
        }
        else
        {
            std::cout << "Unable to disable packet resends." << std::endl;
            return false;
        }

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}

bool setFrameRate(CameraPtr pCam, double frameRate)
{
    try
    {
        // Retrieve GenICam nodemap
        INodeMap& nodeMap = pCam->GetNodeMap();

        // Enable frame rate control if available
        CBooleanPtr ptrFrameRateEnable = nodeMap.GetNode("AcquisitionFrameRateEnable");
        if (IsWritable(ptrFrameRateEnable))
        {
            ptrFrameRateEnable->SetValue(true);
        }
        else
        {
            std::cout << "Unable to enable frame rate control. Aborting..." << std::endl;
            return false;
        }

        // Set the frame rate
        CFloatPtr ptrFrameRate = nodeMap.GetNode("AcquisitionFrameRate");
        if (IsWritable(ptrFrameRate))
        {
            ptrFrameRate->SetValue(frameRate);
            std::cout << "Frame rate set to " << frameRate << " FPS" << std::endl;
        }
        else
        {
            std::cout << "Unable to set frame rate. Aborting..." << std::endl;
            return false;
        }

        return true;
    }
    catch (Spinnaker::Exception& e)
    {
        std::cerr << "Error: " << e.what() << std::endl;
        return false;
    }
}


bool GetStereoProcessingParameters(
	CameraPtr pCam,
	StereoParameters& stereoParameters)
{
	bool result = true;

	INodeMap& nodeMapCamera = pCam->GetNodeMap();

	// populate stereoParameters.disparityScaleFactor from the setting in the camera
	result = result && SpinStereo::GetDisparityScaleFactor(nodeMapCamera, stereoParameters.disparityScaleFactor);

	result = result && SpinStereo::GetRefCameraFocalLength(nodeMapCamera, stereoParameters.focalLength);

	result = result && SpinStereo::GetCameraBaseLine(nodeMapCamera, stereoParameters.baseline);

	result = result && SpinStereo::GetRefCameraImageCenter(nodeMapCamera, stereoParameters.fCurrentCentreRow, stereoParameters.fCurrentCentreCol);

	result = result && SpinStereo::PrintCameraCalibrationParams(nodeMapCamera); 

	return result;
}


class PointCloudGenerator
{
private:
    std::string frameId_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    cv::Mat Q;
    float scaleFactor;
    int decimationFactor = 8;
    float fFx, fBx, fCurrentCentreRow, fCurrentCentreCol;

public:
    PointCloudGenerator(CameraPtr pCam)
    {
        initializeQMatrix(pCam);
    };

    void initializeQMatrix(CameraPtr pCam) {
        INodeMap& nodeMap = pCam->GetNodeMap();
        GetRefCameraFocalLength(nodeMap, fFx);
        GetCameraBaseLine(nodeMap, fBx);
        GetRefCameraImageCenter(nodeMap, fCurrentCentreRow, fCurrentCentreCol);
        GetDisparityScaleFactor(nodeMap, scaleFactor);

        Q = cv::Mat::eye(4, 4, CV_32FC1);
        Q.at<float>(0, 3) = -fCurrentCentreCol;
        Q.at<float>(1, 3) = -fCurrentCentreRow;
        Q.at<float>(2, 3) = fFx;
        Q.at<float>(3, 2) = 1.0 / fBx;
        Q.at<float>(3, 3) = 0;

    }

    bool computePointCloud(const ImagePtr disparityImage,
                            const ImagePtr refImage,
                            StereoParameters stereoParameters,
                            PointCloudParameters pointCloudParameters,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
    {
        cv::Mat disparityImageCV_uint16 = cv::Mat(disparityImage->GetHeight(),
                                                disparityImage->GetWidth(),
                                                CV_16UC1,
                                                disparityImage->GetData(),
                                                disparityImage->GetStride());
                                                        
        unsigned char* refImageData = static_cast<unsigned char*>(refImage->GetData());

        pointCloud->points.reserve((disparityImageCV_uint16.rows / pointCloudParameters.decimationFactor) * 
                                (disparityImageCV_uint16.cols / pointCloudParameters.decimationFactor));

        for (int i = pointCloudParameters.ROIImageCoordinatesVMin; 
                i < std::min(disparityImageCV_uint16.rows, (int)pointCloudParameters.ROIImageCoordinatesVMax); 
                i += pointCloudParameters.decimationFactor) 
        {
            for (int j = pointCloudParameters.ROIImageCoordinatesUMin; 
                    j < std::min(disparityImageCV_uint16.cols, (int)pointCloudParameters.ROIImageCoordinatesUMax); 
                    j += pointCloudParameters.decimationFactor) 
            {
                ushort disparityValue = disparityImageCV_uint16.at<ushort>(i, j);
                
                if (stereoParameters.doInvalidDisparityCheck && disparityValue == stereoParameters.invalidDisparityValue) {
                    continue;  // Skip invalid disparity
                }

                float disparity = disparityValue * stereoParameters.disparityScaleFactor + (float)stereoParameters.minDisparity;
                if (disparity <= 0) {
                    continue;  // Skip invalid or zero disparity
                }

                // Compute 3D point using the known Q matrix components
                float Z = fFx * fBx / disparity;
                float X = ((j - fCurrentCentreCol) * Z) / fFx;
                float Y = ((i - fCurrentCentreRow) * Z) / fFx;

                // Skip points outside the world coordinate bounds
                if (X < pointCloudParameters.ROIWorldCoordinatesXMin || X > pointCloudParameters.ROIWorldCoordinatesXMax ||
                    Y < pointCloudParameters.ROIWorldCoordinatesYMin || Y > pointCloudParameters.ROIWorldCoordinatesYMax ||
                    Z < pointCloudParameters.ROIWorldCoordinatesZMin || Z > pointCloudParameters.ROIWorldCoordinatesZMax) {
                    continue;  // Skip points outside of specified world bounds
                }

                pcl::PointXYZRGB point;
                point.x = X;
                point.y = Y;
                point.z = Z;

                // Assign color from reference image
                int pixel_idx = (i * disparityImageCV_uint16.cols + j) * 3;  
                point.r = refImageData[pixel_idx];
                point.g = refImageData[pixel_idx + 1];
                point.b = refImageData[pixel_idx + 2];

                pointCloud->push_back(point);
            }
        }

        return true;
    }


    bool computePointCloud(const ImagePtr disparityImage,
                            StereoParameters stereoParameters,
                            PointCloudParameters pointCloudParameters,
                            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
    {
        cv::Mat disparityImageCV_uint16 = cv::Mat(disparityImage->GetHeight(),
                                                disparityImage->GetWidth(),
                                                CV_16UC1,
                                                disparityImage->GetData(),
                                                disparityImage->GetStride());

        pointCloud->points.reserve((disparityImageCV_uint16.rows / pointCloudParameters.decimationFactor) * 
                                (disparityImageCV_uint16.cols / pointCloudParameters.decimationFactor));

        for (int i = pointCloudParameters.ROIImageCoordinatesVMin; 
                i < std::min(disparityImageCV_uint16.rows, (int)pointCloudParameters.ROIImageCoordinatesVMax); 
                i += pointCloudParameters.decimationFactor) 
        {
            for (int j = pointCloudParameters.ROIImageCoordinatesUMin; 
                    j < std::min(disparityImageCV_uint16.cols, (int)pointCloudParameters.ROIImageCoordinatesUMax); 
                    j += pointCloudParameters.decimationFactor) 
            {
                ushort disparityValue = disparityImageCV_uint16.at<ushort>(i, j);
                
                if (stereoParameters.doInvalidDisparityCheck && disparityValue == stereoParameters.invalidDisparityValue) {
                    continue;  // Skip invalid disparity
                }

                float disparity = disparityValue * stereoParameters.disparityScaleFactor + (float)stereoParameters.minDisparity;
                if (disparity <= 0) {
                    continue;  // Skip invalid or zero disparity
                }

                // Compute 3D point using the known Q matrix components
                float Z = fFx * fBx / disparity;
                float X = ((j - fCurrentCentreCol) * Z) / fFx;
                float Y = ((i - fCurrentCentreRow) * Z) / fFx;

                // Skip points outside the world coordinate bounds
                if (X < pointCloudParameters.ROIWorldCoordinatesXMin || X > pointCloudParameters.ROIWorldCoordinatesXMax ||
                    Y < pointCloudParameters.ROIWorldCoordinatesYMin || Y > pointCloudParameters.ROIWorldCoordinatesYMax ||
                    Z < pointCloudParameters.ROIWorldCoordinatesZMin || Z > pointCloudParameters.ROIWorldCoordinatesZMax) {
                    continue;  // Skip points outside of specified world bounds
                }

                pcl::PointXYZRGB point;
                point.x = X;
                point.y = Y;
                point.z = Z;

                point.r = 255;
                point.g = 255;
                point.b = 255;

                pointCloud->push_back(point);
            }
        }

        return true;
    }


};


class StereoImagePublisherNode : public rclcpp::Node {
public:
    StereoImagePublisherNode(CameraPtr& pCam,
                            StereoParameters& stereoParameters,
                            PointCloudParameters& pointCloudParameters,
                            std::map<SpinStereo::CameraStreamType, ImagePtr>& imageMap,
                            std::map<SpinStereo::CameraStreamType, int>& streamIndexMap)
        : Node("stereo_image_publisher_node"),
          pCam_(pCam),
          stereoParameters_(stereoParameters),
          pointCloudParameters_(pointCloudParameters),
          imageMap_(imageMap),
          streamIndexMap_(streamIndexMap) {

        // initialize publisher
        initializePublishers();

        // Declare and initialize parameters
        declareParameters(stereoParameters_, pointCloudParameters_);

        // Register a callback for dynamic parameter updates
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&StereoImagePublisherNode::parametersCallback, this, std::placeholders::_1)
        );
    }

    void initializePublishers(){
        // Reset all existing publishers before reinitializing
        raw_left_image_publisher_.reset();
        raw_right_image_publisher_.reset();
        rect_left_image_publisher_.reset();
        rect_right_image_publisher_.reset();
        disparity_image_publisher_.reset();
        point_cloud_publisher_.reset();

        // Conditionally create publishers based on the transmission flags
        if (stereoParameters_.streamTransmitFlags.rawLeftTransmitEnabled) {
            raw_left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Bumblebee_X/raw_left_image", 10);
            RCLCPP_INFO(this->get_logger(), "Raw left image publisher created.");
        }

        if (stereoParameters_.streamTransmitFlags.rawRightTransmitEnabled) {
            raw_right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Bumblebee_X/raw_right_image", 10);
            RCLCPP_INFO(this->get_logger(), "Raw right image publisher created.");
        }

        if (stereoParameters_.streamTransmitFlags.rectLeftTransmitEnabled) {
            rect_left_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Bumblebee_X/rectified_left_image", 10);
            RCLCPP_INFO(this->get_logger(), "Rectified left image publisher created.");
        }

        if (stereoParameters_.streamTransmitFlags.rectRightTransmitEnabled) {
            rect_right_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Bumblebee_X/rectified_right_image", 10);
            RCLCPP_INFO(this->get_logger(), "Rectified right image publisher created.");
        }

        if (stereoParameters_.streamTransmitFlags.disparityTransmitEnabled) {
            disparity_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("Bumblebee_X/disparity_image", 10);
            RCLCPP_INFO(this->get_logger(), "Disparity image publisher created.");
        }

        if (stereoParameters_.doComputePointCloud && stereoParameters_.streamTransmitFlags.disparityTransmitEnabled) {
            point_cloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("Bumblebee_X/point_cloud", 100);
            RCLCPP_INFO(this->get_logger(), "Point cloud publisher created.");
        }
    }

    // Generic function to publish any image (raw or rectified)
    void publishImage(ImagePtr imagePtr, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher, const std::string &encoding, rclcpp::Time timestamp) {
        if (publisher) {
            cv::Mat cvImage = cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(), CV_8UC3, (unsigned char*)imagePtr->GetData(), imagePtr->GetStride());
            cv::cvtColor(cvImage, cvImage, cv::COLOR_RGB2BGR);  // Convert to BGR for ROS

            // Convert to ROS Image message
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding, cvImage).toImageMsg();
            msg->header.stamp = timestamp;
            publisher->publish(*msg);
        }
    }

    // Function to publish disparity image
    void publishDisparityImage(ImagePtr imagePtr, rclcpp::Time timestamp) {
        if (disparity_image_publisher_) {
            cv::Mat disparityImage = cv::Mat(imagePtr->GetHeight(), imagePtr->GetWidth(), CV_16UC1, (unsigned char*)imagePtr->GetData(), imagePtr->GetStride());

            // Convert to ROS Image message
            auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono16", disparityImage).toImageMsg();
            msg->header.stamp = timestamp;
            disparity_image_publisher_->publish(*msg);
        }
    }

    void publishPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, rclcpp::Time timestamp)
    {
        if (point_cloud_publisher_) {
            sensor_msgs::msg::PointCloud2 msg;
            pcl::toROSMsg(*pointCloud, msg);
            msg.header.frame_id = "camera_link";
            msg.header.stamp = timestamp;  // Use ROS 2 clock for timestamp
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

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getRectRightPublisher() {
        return rect_right_image_publisher_;
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr getDisparityPublisher() {
        return disparity_image_publisher_;
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr getPointCloudPublisher() {
        return point_cloud_publisher_;
    }


private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_left_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_right_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_left_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rect_right_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr disparity_image_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_publisher_;

    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    CameraPtr& pCam_;
    StereoParameters& stereoParameters_;
    PointCloudParameters& pointCloudParameters_;
    std::map<SpinStereo::CameraStreamType, int>& streamIndexMap_;
    std::map<SpinStereo::CameraStreamType, ImagePtr>& imageMap_;


    void declareParameters(StereoParameters stereoParameters_, PointCloudParameters pointCloudParameters_) {        
        // stereo parameters
        this->declare_parameter<double>("minDisp", stereoParameters_.minDisparity);
        this->declare_parameter<int>("SGBMP1", stereoParameters_.SGBMP1);
        this->declare_parameter<int>("SGBMP2", stereoParameters_.SGBMP2);
        this->declare_parameter<int>("uniquenessRatio", stereoParameters_.uniquenessRatio);

        // post processing parameters
        this->declare_parameter<bool>("postProcessDisparity", stereoParameters_.postProcessDisparity);
        this->declare_parameter<int>("maxSpeckleSize", stereoParameters_.maxSpeckleSize);
        this->declare_parameter<double>("maxDiff", stereoParameters_.maxDiff);

        // stream transmit parameters
        this->declare_parameter<bool>("rawLeftEnabled", stereoParameters_.streamTransmitFlags.rawLeftTransmitEnabled);
        this->declare_parameter<bool>("rawRightEnabled", stereoParameters_.streamTransmitFlags.rawRightTransmitEnabled);
        this->declare_parameter<bool>("rectLeftEnabled", stereoParameters_.streamTransmitFlags.rectLeftTransmitEnabled);
        this->declare_parameter<bool>("rectRightEnabled", stereoParameters_.streamTransmitFlags.rectRightTransmitEnabled);
        this->declare_parameter<bool>("disparityTransmitEnabled", stereoParameters_.streamTransmitFlags.disparityTransmitEnabled);

        // point cloud parameters
        this->declare_parameter<bool>("pointCloudEnabled", stereoParameters_.doComputePointCloud);
        this->declare_parameter<int>("decimationFactor", pointCloudParameters_.decimationFactor);
        this->declare_parameter<float>("XMin", pointCloudParameters_.ROIWorldCoordinatesXMin);  // X Min
        this->declare_parameter<float>("XMax", pointCloudParameters_.ROIWorldCoordinatesXMax);  // X Max
        this->declare_parameter<float>("YMin", pointCloudParameters_.ROIWorldCoordinatesYMin);  // Y Min
        this->declare_parameter<float>("YMax", pointCloudParameters_.ROIWorldCoordinatesYMax);  // Y Max
        this->declare_parameter<float>("ZMin", pointCloudParameters_.ROIWorldCoordinatesZMin);  // Z Min
        this->declare_parameter<float>("ZMax", pointCloudParameters_.ROIWorldCoordinatesZMax);  // Z Max

        // Region of Interest (ROI) in Image Coordinates
        this->declare_parameter<int>("UMin", pointCloudParameters_.ROIImageCoordinatesUMin);   // U Min
        this->declare_parameter<int>("VMin", pointCloudParameters_.ROIImageCoordinatesVMin);   // V Min
        this->declare_parameter<int>("UMax", pointCloudParameters_.ROIImageCoordinatesUMax);   // U Max
        this->declare_parameter<int>("VMax", pointCloudParameters_.ROIImageCoordinatesVMax);   // V Max
    }

    // Callback to handle parameter updates
    rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> &params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        bool updateSGBMParams = false;
        bool updatePostProcessingParams = false;
        bool updatePublisher = false;

        for (const auto &param : params) {
            if (param.get_name() == "minDisp") {
                double new_value =  param.as_double();
                if (new_value < 0.0 || new_value > 768.0) {
                    RCLCPP_WARN(this->get_logger(), "minDisp out of bounds, must be between 0 and 768. Clamping value.");
                    new_value = std::clamp(new_value, 0.0, 768.0);
                }
                stereoParameters_.minDisparity = new_value;
                RCLCPP_INFO(this->get_logger(), "minDisp updated: %f", stereoParameters_.minDisparity);
                updateSGBMParams = true;
            }
            else if (param.get_name() == "SGBMP1") {
                int new_value = param.as_int();
                if (new_value < 0 || new_value > 240) {
                    RCLCPP_WARN(this->get_logger(), "SGBMP1 out of bounds, must be between 0 and 240. Clamping value.");
                    new_value = std::clamp(new_value, 0, 240);
                }
                if (new_value >= stereoParameters_.SGBMP2){
                    RCLCPP_WARN(this->get_logger(), "SGBMP1 must be less than SGBMP2. Clamping value.");
                    new_value = stereoParameters_.SGBMP2 - 1;                    
                }
                stereoParameters_.SGBMP1 = new_value;
                RCLCPP_INFO(this->get_logger(), "SGBMP1 updated: %d", stereoParameters_.SGBMP1);
                updateSGBMParams = true;
            }
            else if (param.get_name() == "SGBMP2") {
                int new_value = param.as_int();
                if (new_value < 0 || new_value > 240) {
                    RCLCPP_WARN(this->get_logger(), "SGBMP2 out of bounds, must be between 1 and 240. Clamping value.");
                    new_value = std::clamp(new_value, 1, 240);
                }
                if (new_value <= stereoParameters_.SGBMP1){
                    RCLCPP_WARN(this->get_logger(), "SGBMP2 must be greater than SGBMP1. Clamping value.");
                    new_value = stereoParameters_.SGBMP1 + 1;                    
                }
                stereoParameters_.SGBMP2 = new_value;
                RCLCPP_INFO(this->get_logger(), "SGBMP2 updated: %d", stereoParameters_.SGBMP2);
                updateSGBMParams = true;
            }
            else if (param.get_name() == "uniquenessRatio") {
                int new_value = param.as_int();
                if (new_value < 0 || new_value > 100) {
                    RCLCPP_WARN(this->get_logger(), "Uniqueness ratio out of bounds, must be between 0 and 100. Clamping value.");
                    new_value = std::clamp(new_value, 0, 100);
                }
                stereoParameters_.uniquenessRatio = new_value;
                RCLCPP_INFO(this->get_logger(), "uniquenessRatio updated: %d", stereoParameters_.uniquenessRatio);
                updateSGBMParams = true;
            }
            else if (param.get_name() == "postProcessDisparity") {
                stereoParameters_.postProcessDisparity = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Point cloud transmission: %s", stereoParameters_.postProcessDisparity ? "enabled" : "disabled");
            }
            else if (param.get_name() == "maxSpeckleSize") {
                stereoParameters_.maxSpeckleSize = param.as_int();
                RCLCPP_INFO(this->get_logger(), "maxSpeckleSize updated: %d", stereoParameters_.maxSpeckleSize);
            }
            else if (param.get_name() == "maxDiff") {
                stereoParameters_.maxDiff = param.as_double();
                RCLCPP_INFO(this->get_logger(), "maxDiff updated: %f", stereoParameters_.maxDiff);
            }
            else if (param.get_name() == "rawLeftEnabled") {
                stereoParameters_.streamTransmitFlags.rawLeftTransmitEnabled = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "rawLeftEnabled updated: %s", stereoParameters_.streamTransmitFlags.rawLeftTransmitEnabled ? "enabled" : "disabled");
                updatePublisher = true;
            } 
            else if (param.get_name() == "rawRightEnabled") {
                stereoParameters_.streamTransmitFlags.rawRightTransmitEnabled = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "rawRightEnabled updated: %s", stereoParameters_.streamTransmitFlags.rawRightTransmitEnabled ? "enabled" : "disabled");
                updatePublisher = true;
            } 
            else if (param.get_name() == "rectLeftEnabled") {
                stereoParameters_.streamTransmitFlags.rectLeftTransmitEnabled = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "rectLeftEnabled updated: %s", stereoParameters_.streamTransmitFlags.rectLeftTransmitEnabled ? "enabled" : "disabled");
                updatePublisher = true;
            } 
            else if (param.get_name() == "rectRightEnabled") {
                stereoParameters_.streamTransmitFlags.rectRightTransmitEnabled = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "rectRightEnabled updated: %s", stereoParameters_.streamTransmitFlags.rectRightTransmitEnabled ? "enabled" : "disabled");
                updatePublisher = true;
            } 
            else if (param.get_name() == "disparityTransmitEnabled") {
                stereoParameters_.streamTransmitFlags.disparityTransmitEnabled = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "disparityTransmitEnabled updated: %s", stereoParameters_.streamTransmitFlags.disparityTransmitEnabled ? "enabled" : "disabled");
                updatePublisher = true;
            }
            else if (param.get_name() == "pointCloudEnabled") {
                stereoParameters_.doComputePointCloud = param.as_bool();
                RCLCPP_INFO(this->get_logger(), "Point cloud transmission: %s", stereoParameters_.doComputePointCloud ? "enabled" : "disabled");
            }
            else if (param.get_name() == "decimationFactor") {
                int new_value = param.as_int();
                if (new_value < 1) {
                    RCLCPP_WARN(this->get_logger(), "decimationFactor must be greater than 1. Clamping value.");
                    new_value = 1;
                }
                pointCloudParameters_.decimationFactor = new_value;
                RCLCPP_INFO(this->get_logger(), "decimationFactor updated: %d", pointCloudParameters_.decimationFactor);
            }
            else if (param.get_name() == "XMin") {
                pointCloudParameters_.ROIWorldCoordinatesXMin = param.as_double();
                RCLCPP_INFO(this->get_logger(), "XMin updated: %f", pointCloudParameters_.ROIWorldCoordinatesXMin);
            }
            else if (param.get_name() == "XMax") {
                pointCloudParameters_.ROIWorldCoordinatesXMax = param.as_double();
                RCLCPP_INFO(this->get_logger(), "XMax updated: %f", pointCloudParameters_.ROIWorldCoordinatesXMax);
            }
            else if (param.get_name() == "YMin") {
                pointCloudParameters_.ROIWorldCoordinatesYMin = param.as_double();
                RCLCPP_INFO(this->get_logger(), "YMin updated: %f", pointCloudParameters_.ROIWorldCoordinatesYMin);
            }
            else if (param.get_name() == "YMax") {
                pointCloudParameters_.ROIWorldCoordinatesYMax = param.as_double();
                RCLCPP_INFO(this->get_logger(), "YMax updated: %f", pointCloudParameters_.ROIWorldCoordinatesYMax);
            }
            else if (param.get_name() == "ZMin") {
                pointCloudParameters_.ROIWorldCoordinatesZMin = param.as_double();
                RCLCPP_INFO(this->get_logger(), "ZMin updated: %f", pointCloudParameters_.ROIWorldCoordinatesZMin);
            }
            else if (param.get_name() == "ZMax") {
                pointCloudParameters_.ROIWorldCoordinatesZMax = param.as_double();
                RCLCPP_INFO(this->get_logger(), "ZMax updated: %f", pointCloudParameters_.ROIWorldCoordinatesZMax);
            }
            else if (param.get_name() == "UMin") {
                pointCloudParameters_.ROIImageCoordinatesUMin = param.as_int();
                RCLCPP_INFO(this->get_logger(), "UMin updated: %d", pointCloudParameters_.ROIImageCoordinatesUMin);
            }
            else if (param.get_name() == "VMin") {
                pointCloudParameters_.ROIImageCoordinatesVMin = param.as_int();
                RCLCPP_INFO(this->get_logger(), "VMin updated: %d", pointCloudParameters_.ROIImageCoordinatesVMin);
            }
            else if (param.get_name() == "UMax") {
                pointCloudParameters_.ROIImageCoordinatesUMax = param.as_int();
                RCLCPP_INFO(this->get_logger(), "UMax updated: %d", pointCloudParameters_.ROIImageCoordinatesUMax);
            }
            else if (param.get_name() == "VMax") {
                pointCloudParameters_.ROIImageCoordinatesVMax = param.as_int();
                RCLCPP_INFO(this->get_logger(), "VMax updated: %d", pointCloudParameters_.ROIImageCoordinatesVMax);
            }

        }
 
        if (updatePublisher)
        {
            RCLCPP_INFO(this->get_logger(), "Configuring streams");
            try 
            {
                // Reconfigure the camera streams
                if (!SpinStereo::ConfigureCameraStreams(pCam_, imageMap_, true, stereoParameters_.streamTransmitFlags)) {
                    RCLCPP_ERROR(this->get_logger(), "Error during stream reconfiguration: %s", "Could not configure camera streams.");
                    result.successful = false;
                    return result;
                }

                // Reset stream index map after reconfiguration
                if (!SpinStereo::SetStreamIndexMap(pCam_, stereoParameters_.streamTransmitFlags, streamIndexMap_)) {
                    RCLCPP_ERROR(this->get_logger(), "Error during stream reconfiguration: %s", "Could not set stream index map.");
                    result.successful = false;
                    return result;
                }

                initializePublishers();

            } 
            catch (const Spinnaker::Exception &e) {
                RCLCPP_ERROR(this->get_logger(), "Error during stream reconfiguration: %s", e.what());
                result.successful = false;
            }
        }

        
        if(updateSGBMParams)
        {
            // Call the SetSGBM_params after parameters update
            if (!SetSGBM_params()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set stereo parameters.");
                result.successful = false;
            }
        }

        return result;
    }

    // SetSGBM_params function to set stereo parameters
    bool SetSGBM_params() {
        RCLCPP_INFO(this->get_logger(), "Setting SGBM parameters...");

        // Add debugging information before accessing the parameters
        if (!pCam_) {
            RCLCPP_ERROR(this->get_logger(), "Camera pointer (pCam_) is null. Aborting SGBM parameter setting.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "Stereo Parameters: SGBMP1: %d, SGBMP2: %d, minDisparity: %f, uniquenessRatio: %d",
                    stereoParameters_.SGBMP1, stereoParameters_.SGBMP2, stereoParameters_.minDisparity, stereoParameters_.uniquenessRatio);

        // Ensure pCam_ and stereo parameters are valid before calling SetSGBM_params
        if (!SpinStereo::SetSGBM_params(pCam_, stereoParameters_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set SGBM parameters on the camera.");
            return false;
        }

        RCLCPP_INFO(this->get_logger(), "SGBM parameters set successfully.");
        return true;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    StereoParameters stereoParameters;

    // Retrieve singleton reference to system object
    SystemPtr system = System::GetInstance();

    // Retrieve list of cameras from the system
    CameraList camList = system->GetCameras();

    const unsigned int numCameras = camList.GetSize();

    if (numCameras == 0) {
        camList.Clear();
        system->ReleaseInstance();
        cout << "Not enough cameras!" << endl;
        return -1;
    }

    // Create shared pointer to camera
    CameraPtr pCam = camList.GetByIndex(0);
    pCam->Init();
    std::map<SpinStereo::CameraStreamType, int> streamIndexMap;
    std::map<SpinStereo::CameraStreamType, ImagePtr> imageMap;
    bool doSetTransmittedStreams = true;

    // Acquire and publish images
    ConfigureAcquisition(pCam, stereoParameters.streamTransmitFlags, doSetTransmittedStreams, imageMap);
    GetStereoProcessingParameters(pCam, stereoParameters);

    int frameRate = 5;
    setFrameRate(pCam, (double)frameRate);
    disablePacketResends(pCam);

    if (!pCam->IsStreaming()){
        pCam->BeginAcquisition();
    }

    if (!SetStreamIndexMap(pCam, stereoParameters.streamTransmitFlags, streamIndexMap)){
        cout << "Failed to set the stream indices." << endl;
        pCam->EndAcquisition();
        return false;
    }

    PointCloudGenerator pointCloudGenerator(pCam);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
            
    PointCloudParameters pointCloudParameters;
    INodeMap& nodeMapCamera = pCam->GetNodeMap();
    int64_t stereoHeight, stereoWidth;
    if (!GetIntValueFromNode(nodeMapCamera, "StereoHeight", stereoHeight)){
        stereoHeight = 1536;
    }
    if (!GetIntValueFromNode(nodeMapCamera, "StereoWidth", stereoWidth)){
        stereoWidth = 2048;
    }

    pointCloudParameters.decimationFactor = 4;
    pointCloudParameters.ROIImageCoordinatesUMin = 0;                   // U Min
    pointCloudParameters.ROIImageCoordinatesVMin = 0;                   // V Min
    pointCloudParameters.ROIImageCoordinatesUMax = (uint)stereoWidth;   // U Max
    pointCloudParameters.ROIImageCoordinatesVMax = (uint)stereoHeight;  // V Max

    // Region of Interest (ROI) in World Coordinates
    pointCloudParameters.ROIWorldCoordinatesXMin = -10.0f;  // X Min
    pointCloudParameters.ROIWorldCoordinatesXMax = 10.0f;   // X Max
    pointCloudParameters.ROIWorldCoordinatesYMin = -10.0f;  // Y Min
    pointCloudParameters.ROIWorldCoordinatesYMax = 10.0f;   // Y Max
    pointCloudParameters.ROIWorldCoordinatesZMin = 0.0f;    // Z Min
    pointCloudParameters.ROIWorldCoordinatesZMax = 20.0f;   // Z Max

    // Create the ROS2 node for publishing images
    auto publisherNode = std::make_shared<StereoImagePublisherNode>(pCam, stereoParameters, pointCloudParameters, imageMap, streamIndexMap);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(publisherNode);

    rclcpp::Rate rate(10);
    while (rclcpp::ok()){
        // Capture images from the camera
        if (!SpinStereo::GetNextImageGrp(pCam, stereoParameters.streamTransmitFlags, streamIndexMap, 1000, imageMap)){
            continue;  // Continue the loop if images can't be retrieved
        }            
        
        // apply post processing if the parameter is on - will be done in library eventually
        if(stereoParameters.streamTransmitFlags.disparityTransmitEnabled && imageMap[CameraStreamType_DISPARITY] != nullptr && stereoParameters.postProcessDisparity){
            unsigned int width = imageMap[CameraStreamType_DISPARITY]->GetWidth();
            unsigned int height = imageMap[CameraStreamType_DISPARITY]->GetHeight();
            unsigned int stride = imageMap[CameraStreamType_DISPARITY]->GetStride();
            unsigned char* pData = (unsigned char*)imageMap[CameraStreamType_DISPARITY]->GetData();

            // converting Spinnaker image to OpenCV image
            cv::Mat disparityImage = cv::Mat(height, width, CV_16SC1, pData, stride);
            
            cv::filterSpeckles(disparityImage,
                                stereoParameters.invalidDisparityValue,
                                stereoParameters.maxSpeckleSize,
                                stereoParameters.maxDiff / stereoParameters.disparityScaleFactor);

            // convert OpenCV image back to Spinnaker image.
            ImagePtr postProcessedDisparityImage;
            postProcessedDisparityImage = Image::Create(width, height, 0, 0, PixelFormatEnums::PixelFormat_Mono16, pData);

            imageMap[CameraStreamType_DISPARITY] = postProcessedDisparityImage;
        }

        if (stereoParameters.doComputePointCloud && stereoParameters.streamTransmitFlags.disparityTransmitEnabled && imageMap[CameraStreamType_DISPARITY] != nullptr) {    
            pointCloud->clear();
            if (stereoParameters.streamTransmitFlags.rectLeftTransmitEnabled && imageMap[CameraStreamType_RECT_LEFT] != nullptr) {
                pointCloudGenerator.computePointCloud(imageMap[CameraStreamType_DISPARITY],
                                                    imageMap[CameraStreamType_RECT_LEFT],
                                                    stereoParameters,
                                                    pointCloudParameters,
                                                    pointCloud);
            }
            else {
                pointCloudGenerator.computePointCloud(imageMap[CameraStreamType_DISPARITY],
                                                    stereoParameters,
                                                    pointCloudParameters,
                                                    pointCloud);
            }

        }

        auto timestamp = publisherNode->now();

        if (stereoParameters.streamTransmitFlags.rawLeftTransmitEnabled && imageMap[CameraStreamType_RAW_LEFT] != nullptr) {
            publisherNode->publishImage(imageMap[CameraStreamType_RAW_LEFT], publisherNode->getRawLeftPublisher(), "bgr8", timestamp);
        }

        if (stereoParameters.streamTransmitFlags.rawRightTransmitEnabled && imageMap[CameraStreamType_RAW_RIGHT] != nullptr) {
            publisherNode->publishImage(imageMap[CameraStreamType_RAW_RIGHT], publisherNode->getRawRightPublisher(), "bgr8", timestamp);
        }

        if (stereoParameters.streamTransmitFlags.rectLeftTransmitEnabled && imageMap[CameraStreamType_RECT_LEFT] != nullptr) {
            publisherNode->publishImage(imageMap[CameraStreamType_RECT_LEFT], publisherNode->getRectLeftPublisher(), "bgr8", timestamp);
        }

        if (stereoParameters.streamTransmitFlags.rectRightTransmitEnabled && imageMap[CameraStreamType_RECT_RIGHT] != nullptr) {
            publisherNode->publishImage(imageMap[CameraStreamType_RECT_RIGHT], publisherNode->getRectRightPublisher(), "bgr8", timestamp);
        }

        // Publish disparity image
        if (stereoParameters.streamTransmitFlags.disparityTransmitEnabled && imageMap[CameraStreamType_DISPARITY] != nullptr) {
            publisherNode->publishDisparityImage(imageMap[CameraStreamType_DISPARITY], timestamp);
        }

        if (stereoParameters.doComputePointCloud && imageMap[CameraStreamType_DISPARITY] != nullptr && imageMap[CameraStreamType_RECT_LEFT] != nullptr) {
            publisherNode->publishPointCloud(pointCloud, timestamp);
        }

        //rclcpp::spin_some(publisherNode);  // Allow ROS2 to process callbacks
        executor.spin_some();
        rate.sleep();
    }

    // End acquisition
    pCam->EndAcquisition();
    
    ResetGVCPHeartbeat(pCam);

    pCam->DeInit();

    rclcpp::spin(publisherNode);

    camList.Clear();
    system->ReleaseInstance();

    rclcpp::shutdown();
    return 0;
}
