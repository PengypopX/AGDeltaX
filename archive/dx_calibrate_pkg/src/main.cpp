// src/main.cpp
#include "dx_calibrate_pkg/charuco_detector_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create the CharucoDetectorNode
    auto node = std::make_shared<CharucoDetectorNode>();

    node -> setupSubscriptions(); // Now safe to set up ImageTransport subscriptions
    // Spin until Ctrl+C
    rclcpp::spin(node);

    // Shutdown ROS2 cleanly
    rclcpp::shutdown();

    return 0;
}
