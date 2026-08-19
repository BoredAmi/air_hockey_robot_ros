#include "camera_bridge.hpp"

CameraBridge::CameraBridge(Config& config)
: config_(config)
{}

bool CameraBridge::initialize() {
    node_ = std::make_shared<rclcpp::Node>("config_tuner");

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
                   .best_effort()
                   .durability_volatile();

    sub_ = node_->create_subscription<air_hockey_robot_msgs::msg::PuckState>(
        "/camera/raw_stream",
        qos,
        std::bind(&CameraBridge::image_callback, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "waiting for camera messages on /camera/raw_stream (QoS: best_effort, keep_last=1)...");

    return true;
}

void CameraBridge::spinOnce() {
    rclcpp::spin_some(node_);
}

void CameraBridge::image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg) {
    cv::Mat decoded = cv::imdecode(msg->image_frame.data, cv::IMREAD_GRAYSCALE);
    if (decoded.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to decode JPEG frame");
        return;
    }
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_raw_frame_ = decoded;
    has_frame_ = true;
}

cv::Mat CameraBridge::captureRawImage() {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!has_frame_) return cv::Mat();
    return latest_raw_frame_.clone();
}
