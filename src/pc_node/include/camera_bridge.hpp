#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_state.hpp>
#include <opencv2/opencv.hpp>

#include <mutex>
#include <string>

#include "config.hpp"
class CameraBridge {
public:

    explicit CameraBridge(Config& config);

    bool initialize();

    void spinOnce();

    cv::Mat captureRawImage();

    bool hasFrame() const { return has_frame_; }

private:
    void image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg);

    Config& config_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<air_hockey_robot_msgs::msg::PuckState>::SharedPtr sub_;

    std::mutex frame_mutex_;
    cv::Mat latest_raw_frame_;
    bool has_frame_ = false;
};
