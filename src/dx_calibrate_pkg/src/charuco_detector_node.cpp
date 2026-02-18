// src/charuco_detector_node.cpp
#include "dx_calibrate_pkg/charuco_detector_node.hpp"

CharucoDetectorNode::CharucoDetectorNode()
: Node("charuco_detector"),
  dictionary_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50))
{
    // --- Declare parameters ---
    this->declare_parameter<std::string>("camera_frame", "camera_frame");
    this->declare_parameter<std::string>("charuco_frame", "charuco_board");

    camera_frame_ = this->get_parameter("camera_frame").as_string();
    charuco_frame_ = this->get_parameter("charuco_frame").as_string();

    // --- TF broadcaster ---
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // --- Charuco board setup (dummy measurements) ---
    int squares_x = 11;          // number of squares in X direction
    int squares_y = 8;         // number of squares in Y direction
    float square_length = 0.020f; // meters (20 mm)
    float marker_length = 0.015f; // meters (14 mm)

    charuco_board_ = cv::aruco::CharucoBoard::create(
        squares_x, squares_y, square_length, marker_length, dictionary_
    );

    // --- Camera info subscription ---
    cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera_info",
        10,
        std::bind(&CharucoDetectorNode::cameraInfoCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Charuco detector node constructed");
}

// Safe ImageTransport setup
void CharucoDetectorNode::setupSubscriptions()
{
    auto node_ptr = shared_from_this(); // now safe
    image_transport::ImageTransport it(node_ptr);

    image_sub_ = it.subscribe(
        "/image_raw",
        10,
        std::bind(&CharucoDetectorNode::imageCallback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "Image subscription initialized");
}

// --- Callbacks ---
void CharucoDetectorNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    if (camera_info_received_) return;

    camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
    dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, (void*)msg->d.data()).clone();

    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera info received");
}

void CharucoDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    if (!camera_info_received_) return;

    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;

    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> marker_corners;

    cv::aruco::detectMarkers(image, dictionary_, marker_corners, marker_ids);
    if (marker_ids.empty()) return;

    cv::Mat charuco_corners, charuco_ids;
    cv::aruco::interpolateCornersCharuco(
        marker_corners, marker_ids, image,
        charuco_board_, charuco_corners, charuco_ids,
        camera_matrix_, dist_coeffs_
    );

    if (charuco_ids.total() < 4) return;

    RCLCPP_INFO(this->get_logger(), "Detected %lu charuco corners", charuco_ids.total());

    cv::Vec3d rvec, tvec;
    bool valid = cv::aruco::estimatePoseCharucoBoard(
        charuco_corners, charuco_ids, charuco_board_,
        camera_matrix_, dist_coeffs_, rvec, tvec
    );

    if (!valid) return;

    publishTF(rvec, tvec, msg->header.stamp);
}

// --- Publish TF ---
void CharucoDetectorNode::publishTF(const cv::Vec3d& rvec,
                                    const cv::Vec3d& tvec,
                                    const rclcpp::Time& stamp)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);

    tf2::Matrix3x3 tf_R(
        R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
        R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2)
    );

    tf2::Quaternion q;
    tf_R.getRotation(q);

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = stamp;
    tf_msg.header.frame_id = camera_frame_;
    tf_msg.child_frame_id = charuco_frame_;

    tf_msg.transform.translation.x = tvec[0];
    tf_msg.transform.translation.y = tvec[1];
    tf_msg.transform.translation.z = tvec[2];

    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}
