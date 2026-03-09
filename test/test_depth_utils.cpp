#include <gtest/gtest.h>

#include <opencv2/core.hpp>

#include "stereo_image_publisher/depth_utils.hpp"

TEST(DepthUtilsTest, ComputeDepthImageHandlesValidAndInvalidPixels) {
  cv::Mat disparity(2, 3, CV_16UC1);
  disparity.at<uint16_t>(0, 0) = 0;
  disparity.at<uint16_t>(0, 1) = 256;
  disparity.at<uint16_t>(0, 2) = 512;
  disparity.at<uint16_t>(1, 0) = 128;
  disparity.at<uint16_t>(1, 1) = 64;
  disparity.at<uint16_t>(1, 2) = 1;

  const cv::Mat depth = bumblebee_ros::depth_utils::compute_depth_image(
      disparity,
      1.0f / 256.0f,
      0.0f,
      2.0f,
      1.0f,
      10.0f);

  ASSERT_EQ(depth.type(), CV_32FC1);
  EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 0.0f);
  EXPECT_NEAR(depth.at<float>(0, 1), 2.0f, 1e-6f);
  EXPECT_NEAR(depth.at<float>(0, 2), 1.0f, 1e-6f);
  EXPECT_NEAR(depth.at<float>(1, 0), 4.0f, 1e-6f);
  EXPECT_NEAR(depth.at<float>(1, 1), 8.0f, 1e-6f);
  EXPECT_FLOAT_EQ(depth.at<float>(1, 2), 0.0f);
}

TEST(DepthUtilsTest, ComputeDepthImageRejectsNonPositiveDisparity) {
  cv::Mat disparity(1, 2, CV_16UC1);
  disparity.at<uint16_t>(0, 0) = 128;
  disparity.at<uint16_t>(0, 1) = 64;

  const cv::Mat depth = bumblebee_ros::depth_utils::compute_depth_image(
      disparity,
      1.0f / 256.0f,
      -0.5f,
      2.0f,
      1.0f,
      10.0f);

  EXPECT_FLOAT_EQ(depth.at<float>(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(depth.at<float>(0, 1), 0.0f);
}

TEST(DepthUtilsTest, DepthToColormapMasksInvalidPixels) {
  cv::Mat depth = cv::Mat::zeros(2, 2, CV_32FC1);
  depth.at<float>(0, 1) = 1.0f;
  depth.at<float>(1, 0) = 2.0f;

  const cv::Mat colormap = bumblebee_ros::depth_utils::depth_to_colormap(depth);

  ASSERT_EQ(colormap.type(), CV_8UC3);
  EXPECT_EQ(colormap.rows, depth.rows);
  EXPECT_EQ(colormap.cols, depth.cols);

  const cv::Vec3b invalid_px = colormap.at<cv::Vec3b>(0, 0);
  EXPECT_EQ(invalid_px[0], 0);
  EXPECT_EQ(invalid_px[1], 0);
  EXPECT_EQ(invalid_px[2], 0);

  const cv::Vec3b valid_px = colormap.at<cv::Vec3b>(0, 1);
  EXPECT_GT(static_cast<int>(valid_px[0]) + static_cast<int>(valid_px[1]) +
                static_cast<int>(valid_px[2]),
            0);
}

TEST(DepthUtilsTest, DepthToColormapAllZerosIsBlack) {
  cv::Mat depth = cv::Mat::zeros(2, 2, CV_32FC1);
  const cv::Mat colormap = bumblebee_ros::depth_utils::depth_to_colormap(depth);
  ASSERT_EQ(colormap.type(), CV_8UC3);
  EXPECT_EQ(cv::countNonZero(colormap.reshape(1)), 0);
}
