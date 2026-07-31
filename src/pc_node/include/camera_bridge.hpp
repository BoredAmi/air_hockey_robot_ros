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
    cv::Mat captureImage();      

    cv::Point2f detectPuck(const cv::Mat& grayImage);
    cv::Point2f imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight);
    cv::Point2f TableToImageCoordinates(cv::Point2f tablePoint, int imageWidth, int imageHeight);

    void tableFound(bool found);

    int getCroppedWidth() const { return croppedWidth_; }
    int getCroppedHeight() const { return croppedHeight_; }

    bool hasFrame() const { return has_frame_; }

private:
    void image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg);
    cv::RotatedRect detectTable(cv::Mat& image);

    bool loadCachedPerspective(const std::string& filename = "table_perspective.yml");
    bool saveCachedPerspective(const std::string& filename = "table_perspective.yml");

    Config& config_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<air_hockey_robot_msgs::msg::PuckState>::SharedPtr sub_;

    std::mutex frame_mutex_;
    cv::Mat latest_raw_frame_;
    bool has_frame_ = false;

    cv::Rect tableBoundingRect_;
    cv::Size tableOutputSize_;
    cv::Mat tablePerspectiveMatrix_;   
    bool tableDetected_ = false;
    bool tablePerspectiveCached_ = false;

    int croppedWidth_;
    int croppedHeight_;
};
