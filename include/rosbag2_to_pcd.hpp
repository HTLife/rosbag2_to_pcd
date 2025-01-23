// rosbag2_to_pcd.hpp

// SPDX-License-Identifier: Apache-2.0
// Copyright 2023 M. Fatih Cırıt
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>

namespace rosbag2_to_pcd
{

class Rosbag2ToPcdNode : public rclcpp::Node
{
  public:
    /**
     * @brief Construct the Rosbag2ToPcdNode using command-line arguments.
     *
     * @param argc Argument count from the command-line.
     * @param argv Argument vector from the command-line.
     */
    Rosbag2ToPcdNode(int argc, char **argv, std::atomic<bool> &spin_flag);

  private:
    std::atomic<bool> &spin_flag_;

    /**
     * @brief Processes the bag file to extract and save PointCloud2 messages as
     * PCD files.
     *
     * @param path_bag Path to the bag file.
     * @param topic_cloud The PointCloud2 topic to extract. If empty,
     * auto-detects a single topic.
     * @param path_pcds Directory to save the PCD files. Defaults to
     * "<path_bag>_pcds" if empty.
     * @throws std::runtime_error if no suitable topic is found or if multiple
     * topics are found without specifying.
     */
    void process_bag_file(const std::string &path_bag,
                          const std::string &topic_cloud,
                          const std::string &path_pcds);

    /**
     * @brief Detects the format of the PointCloud2 message based on its fields.
     *
     * @param msg_cloud The PointCloud2 message to analyze.
     * @return std::string The detected point cloud format (e.g., xyz, xyzrgb,
     * xyzi, etc.).
     * @throws std::runtime_error if an unsupported format is detected.
     */
    std::string
    detect_point_cloud_format(const sensor_msgs::msg::PointCloud2 &msg_cloud);

    /**
     * @brief Converts and saves a PointCloud2 message to a PCD file based on
     * the detected format.
     *
     * @param msg_cloud The PointCloud2 message to convert.
     * @param format The detected format of the point cloud (e.g., xyz, xyzrgb,
     * xyzi, etc.).
     * @param filename The output PCD file name.
     */
    void save_point_cloud_to_pcd(const sensor_msgs::msg::PointCloud2 &msg_cloud,
                                 const std::string &format,
                                 const std::string &filename);
};

} // namespace rosbag2_to_pcd