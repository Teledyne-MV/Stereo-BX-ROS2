
# stereo_image_publisher

This package provides a stereo image publisher for Spinnaker-based Bumblebee X stereo cameras, which captures stereo images, and computes disparity maps on board. The package allows users to publish the captured images and disparity in ROS2. Additionally, the package also demonstrates filtering the disparity image, and generating point cloud.

## Table of Contents
- [stereo\_image\_publisher](#stereo_image_publisher)
  - [Table of Contents](#table-of-contents)
  - [System Requirements](#system-requirements)
  - [Installation Steps](#installation-steps)
    - [1. Install ROS 2 Humble](#1-install-ros-2-humble)
    - [2. Install Dependencies](#2-install-dependencies)
      - [OpenCV](#opencv)
      - [PCL (Point Cloud Library)](#pcl-point-cloud-library)
      - [Spinnaker SDK (for FLIR cameras)](#spinnaker-sdk-for-flir-cameras)
    - [3. Clone the Repository](#3-clone-the-repository)
    - [4. Build the Package](#4-build-the-package)
  - [Running the Application](#running-the-application)
    - [1. Run the Node](#1-run-the-node)
    - [2. Visualize in Rviz](#2-visualize-in-rviz)
    - [3. Control Parameters with RQT](#3-control-parameters-with-rqt)
  - [Troubleshooting](#troubleshooting)
  - [Additional Resources](#additional-resources)

---

## System Requirements
- **OS**: Ubuntu 22.04 LTS
- **ROS 2 Distribution**: Humble Hawksbill
- **Compiler**: GCC 9 or newer
- **Hardware**: BumbleBee X FLIR cameras
- **Spinnaker SDK**: Spinnaker SDK v4.1.0.3xx or above

---

## Installation Steps

### 1. Install ROS 2 Humble
Follow these steps to install ROS 2 Humble on Ubuntu 22.04:

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup Sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
# *Warning: apt-key is deprecated. Manage keyring files in trusted.gpg.d instead (see apt-key(8)).
sudo apt-add-repository http://packages.ros.org/ros2/ubuntu

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop

# Source the ROS 2 environment
source /opt/ros/humble/setup.bash

# Install development tools and dependencies for building ROS packages
sudo apt install python3-rosdep python3-colcon-common-extensions
sudo rosdep init
rosdep update
```

### Optional: Configure ROS Sourcing

To make the ROS environment available automatically in every new terminal, you can add the ROS source command to your `.bashrc` file.

This step is optional but can be convenient.

1. Open .bashrc with a text editor:
```bash
nano ~/.bashrc
```

2. Add the following line at the end of the file:
```bash
source /opt/ros/humble/setup.bash
```

3. Save the changes and close the editor.

4. Reload .bashrc for the changes to take effect immediately:
```bash
source ~/.bashrc
```

#### Note: A reboot is generally not required for these changes to take effect, but reloading .bashrc as shown above ensures it’s set up without rebooting.

If sourcing ROS is not added to `.bashrc`, ROS should be sourced manually on every new terminal with:

```bash
# source ROS environment
source /opt/ros/humble/setup.bash
```

### 2. Install Dependencies

#### OpenCV
```bash
sudo apt install libopencv-dev
sudo apt-get install ros-humble-cv-bridge
```

#### PCL (Point Cloud Library)
```bash
sudo apt install libpcl-dev
sudo apt-get install ros-humble-pcl-ros
```

#### Spinnaker SDK (for Teledyne FLIR cameras)
Download and install the Spinnaker SDK from FLIR's official website.

#### Note: To use BumbleBee X stereo camera, Spinnaker v4.1.0.3xx or above is required.

1. Download the SDK: [Spinnaker SDK Download](https://www.flir.com/products/spinnaker-sdk/)
2. Follow the installation instructions provided with the SDK.

### 3. Clone the Repository

```bash
# Create a workspace for your ROS packages
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the stereo_image_publisher repository
git clone https://github.com/Teledyne-MV/Stereo-BX-ROS2.git
```

### 4. Build the Package

```bash
cd ~/ros2_ws/
colcon build --packages-select bumblebee_ros
source install/setup.bash
```

Note: If there have been any changes to the stereo publisher source files or Spinnaker version, you may want to clean and rebuild the publisher. The following package and commands can be used to clean the workspace. Rerun the build command above after cleaning the workspace.

```bash
sudo apt install python3-colcon-clean
colcon clean workspace
```

---

## Running the Application

### 1. Run the Node

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Run the stereo image publisher node
ros2 run bumblebee_ros stereo_image_publisher
```

If there are multiple BumbleBee X cameras plugged into the system, the wrapper will select the first one it detects.
To specify which BumbleBee X camera to use with the publisher, the serial number can be passed as an argument. 

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Run the stereo image publisher node
ros2 run bumblebee_ros stereo_image_publisher <serial_number>
```

### 2. Visualize in Rviz

1. Open a new terminal.

2. Open Rviz using the following command:

   ```bash
      # Run RViz2 to visualize the published images and point cloud
      rviz2
   ```
   ![rviz2](images/rviz_window.png)

3. Add the topics for image and point cloud:
   - `/Bumblebee_X/<serial_number>/raw_left_image`
   - `/Bumblebee_X/<serial_number>/disparity_image`
   - `/Bumblebee_X/<serial_number>/point_cloud`

4. Static Transform Pointcloud:
   - If the camera is upright and facing a target, we treat the base as the floor for viewing purposes. The following commands will configure viewing of the pointcloud in RViz.

      ```bash
         # -1.5708 rad = -90 degrees
         # 1.5 meters
         ros2 run tf2_ros static_transform_publisher 0 0 1.5 0 0 -1.5708 base_link camera_link  
      ```

5. Pointcloud Fixed Frame:
   - Change this to base_link.
   ![base_link](images/rviz_fixed_frame.png)

### 3. Control Parameters with RQT

1. Open a new terminal and run RQT to dynamically control the stereo image publisher parameters:
   
   ```bash
      # Run RQT to read/write dynamic parameters.
      rqt
   ```

2. On the top, select Plugins > Configuration > Dynamic Reconfigure.

3. In RQT on the lefthand side, there should be a selectable node called "stereo_image_publisher_<serial_number>". If not, make sure the stereo image publisher is running, and try refreshing the list.

4. Select the "stereo_image_publisher_<serial_number>" node. Parameters for the publisher will populate on the righthand side and can be adjusted.

#### Note: Increaing the frame rate might cause some unexpected behaviour in RViz like dropping image frames and point cloud from display. 

---

## Troubleshooting

1. **Spinnaker SDK Initialization Issues:**
   - Ensure the SPINNAKER_GENTL64_CTI environment variable is valid:
     ```bash
     printenv | grep SPINNAKER_GENTL64_CTI
     
     # SPINNAKER_GENTL64_CTI should point to an existing /opt/spinnaker/lib/spinnaker-gentl/Spinnaker_GenTL.cti
     ```

   - Check that the camera is able to enumerate by running the Spinnaker Enumeration example:
     ```bash
     Enumeration # Located at /opt/spinnaker/bin/Enumeration, which should be in PATH environment variable provided Spinnaker was installed successfully.

     # Looks like the following if camera is not enumerating:
     # Application build date: Oct  2 2024 18:00:48

     # Spinnaker library version: 4.1.0.309

     # Number of interfaces detected: 3

     # Number of cameras detected: 0

     # Not enough cameras!
     # Done! Press Enter to exit...
     ```

2. **ROS2 Not Found:**
   - Ensure ROS2 is sourced:
     ```bash
     source /opt/ros/humble/setup.bash
     ```

3. **Missing Dependencies:**
   - Ensure all required ROS2 and Spinnaker libraries are installed.

4. **Stream Issues:**
   - Ensure the camera is properly initialized and all required streams are enabled via dynamic reconfiguration.

---

## Additional Resources

- [ROS2 Documentation](https://docs.ros.org/en/humble/index.html)
- [Spinnaker SDK Documentation](https://www.flir.com/support-center/iis/machine-vision/downloads/spinnaker-sdk-and-firmware-download/)
- [OpenCV Documentation](https://docs.opencv.org/master/)

---
