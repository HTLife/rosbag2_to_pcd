# ROS2 bag2pcd

The `bag2pcd` package provides functionality to convert point cloud data from a ROS2 rosbag file to PCD (Point
Cloud Data) files.
For each point cloud message in the specified topic, the package creates an individual PCD file.

**Alternative (much more capable) tool:**
- https://github.com/AIT-Assistive-Autonomous-Systems/ros2bag_tools#export

## Installation

To install, follow the [ROS 2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and then execute:

```bash
mkdir -p ~/rosbag2pcd_ws/src
cd ~/rosbag2pcd_ws/src
git clone https://github.com/xmfcx/rosbag2_to_pcd.git
cd ~/rosbag2pcd_ws

sudo apt update
rosdep init
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO

colcon build --symlink-install  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1
```

## Usage

Execute the following commands:

```bash
. ~/rosbag2pcd_ws/install/setup.bash
ros2 run bag2pcd bag2pcd ~/336l/rosbag2_2025_01_17-15_52_29
```