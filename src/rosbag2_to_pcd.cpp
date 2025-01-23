// rosbag2_to_pcd.cpp

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
#include <iostream>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_storage/storage_options.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include "progress_bar.hpp"
#include <chrono>
#include <thread>

namespace rosbag2_to_pcd
{

namespace fs = std::filesystem;

Rosbag2ToPcdNode::Rosbag2ToPcdNode(int argc, char **argv,
                                   std::atomic<bool> &spin_flag)
    : Node("rosbag2_to_pcd")
    , spin_flag_(spin_flag)
{
    if (argc < 2 || argc > 4)
    {
        RCLCPP_INFO(get_logger(), "argc: %d", argc);
        RCLCPP_ERROR(get_logger(),
                     "Usage: ros2 run bag2pcd bag2pcd <rosbag_folder> "
                     "[<topic>] [<output_directory>]");
        spin_flag_ = false;
        return;
    }

    const std::string path_bag = fs::absolute(argv[1]).string();
    const std::string topic_cloud = (argc > 2) ? argv[2] : "";
    const std::string path_pcds = (argc > 3) ? argv[3] : path_bag + "_pcds";

    try
    {
        fs::create_directories(path_pcds);
        process_bag_file(path_bag, topic_cloud, path_pcds);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR_STREAM(get_logger(), "Error: " << e.what());
        spin_flag_ = false;
    }
}

void Rosbag2ToPcdNode::process_bag_file(const std::string &path_bag,
                                        const std::string &topic_cloud,
                                        const std::string &path_pcds)
{
    size_t total_messages = 0; // Progress bar

    rosbag2_cpp::Reader reader;
    /// open
    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = path_bag;
    storage_options.storage_id = "sqlite3"; // Specify the storage ID explicitly

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    reader.open(storage_options, converter_options);
    /// open

    const auto &topics = reader.get_metadata().topics_with_message_count;
    std::string selected_topic = topic_cloud;

    if (selected_topic.empty())
    {
        for (const auto &topic : topics)
        {
            if (topic.topic_metadata.type == "sensor_msgs/msg/PointCloud2")
            {
                if (!selected_topic.empty())
                {
                    throw std::runtime_error(
                        "Multiple PointCloud2 topics found. Please specify one "
                        "explicitly.");
                }
                selected_topic = topic.topic_metadata.name;
                total_messages = topic.message_count; // Get the total messages
                                                      // for the selected topic
            }
        }

        if (selected_topic.empty())
        {
            throw std::runtime_error(
                "No PointCloud2 topic found in the bag file.");
        }
        std::cout << "Detected PointCloud2 topic: " << selected_topic
                  << std::endl;
    }

    rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serialization;
    std::string point_cloud_format;
    bool format_detected = false;
    int ctr_msg_cloud = 0;

    ProgressBar bar(total_messages); // Progress bar
    while (reader.has_next())
    {

        auto bag_message = reader.read_next();

        if (bag_message->topic_name == selected_topic)
        {
            auto msg_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
            rclcpp::SerializedMessage extracted_serialized_msg(
                *bag_message->serialized_data);
            serialization.deserialize_message(&extracted_serialized_msg,
                                              msg_cloud.get());

            if (!format_detected)
            {
                point_cloud_format = detect_point_cloud_format(*msg_cloud);
                format_detected = true;
                std::cout << "Detected point cloud format: "
                          << point_cloud_format << std::endl;
            }

            std::stringstream ss_timestamp;
            ss_timestamp << msg_cloud->header.stamp.sec << "." << std::setw(9)
                         << std::setfill('0')
                         << msg_cloud->header.stamp.nanosec;
            const std::string filename =
                path_pcds + "/" + ss_timestamp.str() + ".pcd";

            save_point_cloud_to_pcd(*msg_cloud, point_cloud_format, filename);

            ++ctr_msg_cloud;
            bar.update(ctr_msg_cloud); // Progress bar
        }
    }
    std::cout << "Output folder: " << path_pcds << std::endl;

    spin_flag_ = false;
}

std::string Rosbag2ToPcdNode::detect_point_cloud_format(
    const sensor_msgs::msg::PointCloud2 &msg_cloud)
{
    std::unordered_set<std::string> field_names;
    for (const auto &field : msg_cloud.fields)
    {
        field_names.insert(field.name);
    }

    if (field_names == std::unordered_set<std::string>{"x", "y", "z"})
    {
        return "xyz";
    }
    else if (field_names ==
             std::unordered_set<std::string>{"x", "y", "z", "rgb"})
    {
        return "xyzrgb";
    }
    else if (field_names ==
             std::unordered_set<std::string>{"x", "y", "z", "intensity"})
    {
        return "xyzi";
    }

    throw std::runtime_error("Unsupported point cloud format detected!");
}

void Rosbag2ToPcdNode::save_point_cloud_to_pcd(
    const sensor_msgs::msg::PointCloud2 &msg_cloud, const std::string &format,
    const std::string &filename)
{
    if (format == "xyz")
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(msg_cloud, *cloud);
        pcl::io::savePCDFileBinary(filename, *cloud);
    }
    else if (format == "xyzrgb")
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(msg_cloud, *cloud);
        pcl::io::savePCDFileBinary(filename, *cloud);
    }
    else if (format == "xyzi")
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
            new pcl::PointCloud<pcl::PointXYZI>);
        pcl::io::savePCDFileBinary(filename, *cloud);
    }
    else
    {
        throw std::runtime_error("Unsupported point cloud format for saving!");
    }
}

} // namespace rosbag2_to_pcd
