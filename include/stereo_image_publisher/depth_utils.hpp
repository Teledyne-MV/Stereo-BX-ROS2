#ifndef STEREO_IMAGE_PUBLISHER__DEPTH_UTILS_HPP_
#define STEREO_IMAGE_PUBLISHER__DEPTH_UTILS_HPP_

#include <opencv2/core.hpp>

namespace bumblebee_ros {
namespace depth_utils {

cv::Mat compute_depth_image(const cv::Mat& disparity_image,
                            float disparity_scale_factor,
                            float scan3d_coordinate_offset,
                            float focal_length,
                            float baseline,
                            float max_depth_meters);

cv::Mat depth_to_colormap(const cv::Mat& depth_image);

}  // namespace depth_utils
}  // namespace bumblebee_ros

#endif  // STEREO_IMAGE_PUBLISHER__DEPTH_UTILS_HPP_
