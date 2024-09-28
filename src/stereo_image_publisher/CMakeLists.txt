cmake_minimum_required(VERSION 3.22)
project(stereo_image_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)

set(Spinnaker_PATH "/home/shubam/zone2/development/spinnaker/builds/dev/4.1.0.304_SDK-8995_2024-09-26_13-33-01/Linux/x64_jammy")

set(Spinnaker_INCLUDE_DIRS "${Spinnaker_PATH}/include")
set(Spinnaker_LIBRARIES "${Spinnaker_PATH}/lib/libSpinnaker.so")



# Include directories
include_directories(
  include
  ${Spinnaker_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# Add the executable with all the source files
add_executable(stereo_image_publisher
  src/stereo_image_publisher.cpp
  src/SpinStereoHelper.cpp
  src/StereoParameters.cpp
)

# Link libraries
target_link_libraries(stereo_image_publisher
  ${Spinnaker_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${pcl_conversions_LIBRARIES}
  ${pcl_ros_LIBRARIES}
)

ament_target_dependencies(stereo_image_publisher
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  PCL
  pcl_conversions
  pcl_ros
)

# Install the executable
install(TARGETS
  stereo_image_publisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

