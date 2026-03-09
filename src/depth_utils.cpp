#include "stereo_image_publisher/depth_utils.hpp"

#include <cmath>

#include <opencv2/imgproc.hpp>

namespace bumblebee_ros {
namespace depth_utils {

cv::Mat compute_depth_image(const cv::Mat& disparity_image,
                            float disparity_scale_factor,
                            float scan3d_coordinate_offset,
                            float focal_length,
                            float baseline,
                            float max_depth_meters) {
  if (disparity_image.empty() || disparity_image.type() != CV_16UC1) {
    return cv::Mat();
  }

  const float denominator = 256.0f + scan3d_coordinate_offset;
  const float min_depth =
      denominator > 0.0f ? (focal_length * baseline) / denominator : 0.0f;

  cv::Mat depth_image = cv::Mat::zeros(disparity_image.size(), CV_32FC1);
  for (int row = 0; row < disparity_image.rows; ++row) {
    for (int col = 0; col < disparity_image.cols; ++col) {
      const uint16_t d = disparity_image.at<uint16_t>(row, col);
      if (d == 0U) {
        depth_image.at<float>(row, col) = 0.0f;
        continue;
      }

      const float disparity =
          static_cast<float>(d) * disparity_scale_factor +
          scan3d_coordinate_offset;
      if (disparity <= 0.0f) {
        depth_image.at<float>(row, col) = 0.0f;
        continue;
      }

      const float depth = (focal_length * baseline) / disparity;
      if (!std::isfinite(depth) || depth > max_depth_meters || depth < min_depth) {
        depth_image.at<float>(row, col) = 0.0f;
      } else {
        depth_image.at<float>(row, col) = depth;
      }
    }
  }
  return depth_image;
}

cv::Mat depth_to_colormap(const cv::Mat& depth_image) {
  if (depth_image.empty()) {
    return cv::Mat();
  }

  cv::Mat depth_float;
  if (depth_image.type() == CV_32FC1) {
    depth_float = depth_image;
  } else {
    depth_image.convertTo(depth_float, CV_32FC1);
  }

  cv::Mat normalized_depth;
  cv::normalize(depth_float, normalized_depth, 0, 1, cv::NORM_MINMAX);

  cv::Mat depth_8bits;
  normalized_depth.convertTo(depth_8bits, CV_8U, 255.0);

  cv::Mat colormap;
  cv::applyColorMap(depth_8bits, colormap, cv::COLORMAP_HSV);

  const cv::Mat invalid_mask = (depth_float == 0.0f);
  colormap.setTo(cv::Scalar(0, 0, 0), invalid_mask);
  return colormap;
}

}  // namespace depth_utils
}  // namespace bumblebee_ros
