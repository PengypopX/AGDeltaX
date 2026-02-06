// include/dx_calibrate_pkg/charuco_detector_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <image_transport/image_transport.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

class CharucoDetectorNode : public rclcpp::Node
{
public:
    CharucoDetectorNode();
      // Safe subscription setup (needs shared_from_this)
    void setupSubscriptions();
private:
    // Callbacks
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
    void publishTF(const cv::Vec3d& rvec,
                   const cv::Vec3d& tvec,
                   const rclcpp::Time& stamp);

    // ROS members
    image_transport::Subscriber image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Camera parameters
    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_ = false;

    // Frame names
    std::string camera_frame_;
    std::string charuco_frame_;

    // Charuco board
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::CharucoBoard> charuco_board_;
};