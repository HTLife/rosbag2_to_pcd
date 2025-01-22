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

#include "rosbag2_to_pcd.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <exception>
#include <filesystem>
#include <string>
#include <unordered_set>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rosbag2_to_pcd
{

Rosbag2ToPcdNode::Rosbag2ToPcdNode(int argc, char **argv)
: Node("rosbag2_to_pcd")
{
  RCLCPP_INFO(get_logger(), "Starting rosbag2_to_pcd node...");

  // Ensure correct number of arguments
  if (argc != 4) {
    RCLCPP_ERROR(
      get_logger(),
      "Usage: ros2 run pcl_ros bag_to_pcd <input_file.bag> <topic> <output_directory>");
    rclcpp::shutdown();
    return;
  }

  const std::string path_bag = argv[1];
  const std::string topic_cloud = argv[2];
  const std::string path_pcds = argv[3];

  // Create output directory
  std::filesystem::create_directories(path_pcds);

  rosbag2_cpp::Reader reader;
  try {
    reader.open(path_bag);
  } catch (const std::exception &e) {
    RCLCPP_ERROR_STREAM(get_logger(), "Error opening bag file: " << e.what());
    rclcpp::shutdown();
    return;
  }

  const auto &topics = reader.get_metadata().topics_with_message_count;
  const auto iter_topic =
    std::find_if(topics.begin(), topics.end(), [&topic_cloud](const auto &topic) {
      return topic.topic_metadata.name == topic_cloud;
    });

  if (iter_topic == topics.end()) {
    RCLCPP_ERROR(get_logger(), "Topic not found in the bag file.");
    rclcpp::shutdown();
    return;
  }

  int ctr_msg_cloud = 1;
  rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;

  std::string point_cloud_format;
  bool format_detected = false;

  while (reader.has_next()) {
    if (!rclcpp::ok()) {
      RCLCPP_INFO(get_logger(), "Interrupted by user, shutting down...");
      rclcpp::shutdown();
      break;
    }

    auto bag_message = reader.read_next();

    if (bag_message->topic_name == topic_cloud) {
      auto msg_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      rclcpp::SerializedMessage extracted_serialized_msg(*bag_message->serialized_data);
      serialization.deserialize_message(&extracted_serialized_msg, msg_cloud.get());

      if (!format_detected) {
        point_cloud_format = detect_point_cloud_format(*msg_cloud);
        format_detected = true;
        RCLCPP_INFO_STREAM(get_logger(), "Detected point cloud format: " << point_cloud_format);
      }

      std::stringstream ss_timestamp;
      ss_timestamp 
        << msg_cloud->header.stamp.sec 
        << "."  
        << std::setw(9) << std::setfill('0') 
        << msg_cloud->header.stamp.nanosec;

      std::string timestamp = ss_timestamp.str();
      std::string filename = path_pcds + "/" + timestamp + ".pcd";

      save_point_cloud_to_pcd(*msg_cloud, point_cloud_format, filename);
      ctr_msg_cloud++;
    }
  }

  RCLCPP_INFO(get_logger(), "Finished converting bag file to pcd files.");
  rclcpp::shutdown();
}

std::string Rosbag2ToPcdNode::detect_point_cloud_format(const sensor_msgs::msg::PointCloud2 &msg_cloud)
{
  std::unordered_set<std::string> field_names;
  for (const auto &field : msg_cloud.fields) {
    field_names.insert(field.name);
  }

  if (field_names == std::unordered_set<std::string>{"x", "y", "z"}) {
    return "xyz";
  } else if (field_names == std::unordered_set<std::string>{"x", "y", "z", "rgb"}) {
    return "xyzrgb";
  } else if (field_names == std::unordered_set<std::string>{"x", "y", "z", "intensity"}) {
    return "xyzi";
  }

  throw std::runtime_error("Unsupported point cloud format detected!");
}

void Rosbag2ToPcdNode::save_point_cloud_to_pcd(
  const sensor_msgs::msg::PointCloud2 &msg_cloud, 
  const std::string &format, 
  const std::string &filename)
{
  if (format == "xyz") {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(msg_cloud, *cloud);
    pcl::io::savePCDFileBinary(filename, *cloud);
  } else if (format == "xyzrgb") {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(msg_cloud, *cloud);
    pcl::io::savePCDFileBinary(filename, *cloud);
  } else if (format == "xyzi") {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::io::savePCDFileBinary(filename, *cloud);
  } else {
    throw std::runtime_error("Unsupported point cloud format for saving!");
  }
}

}  // namespace rosbag2_to_pcd
