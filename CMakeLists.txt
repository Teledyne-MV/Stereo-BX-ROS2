cmake_minimum_required(VERSION 3.22)
project(stereo_image_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Adjust the path to your spinnaker installed location
set(Spinnaker_PATH "/opt/spinnaker")
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
  src/spin_stereo_helper.cpp
  src/stereo_parameters.cpp
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
  image_transport
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

