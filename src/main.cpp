#include "rosbag2_to_pcd.hpp"
#include <atomic>
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char **argv)
{
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    std::atomic<bool> spin_flag(true);

    try
    {
        auto node = std::make_shared<rosbag2_to_pcd::Rosbag2ToPcdNode>(
            argc, argv, spin_flag);

        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node);

        while (spin_flag && rclcpp::ok())
        {
            executor.spin_some(); // Spin while the flag is true
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rosbag2_to_pcd"),
                     "Exception caught: %s", e.what());
    }

    // Shutdown ROS 2
    rclcpp::shutdown();

    return 0;
}