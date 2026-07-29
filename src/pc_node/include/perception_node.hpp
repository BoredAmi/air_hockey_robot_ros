#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_state.hpp>
#include <air_hockey_robot_msgs/msg/puck_detection.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>

namespace pc_node {

struct DetectorConfig {
    int TABLE_DETECT_THRESHOLD;
    int PUCK_THRESHOLD;
    double PUCK_MIN_AREA;
    double PUCK_MAX_AREA;
    double PHYSICAL_TABLE_WIDTH;   // mm
    double PHYSICAL_TABLE_HEIGHT;  // mm
    int TABLE_WIDTH;               // px (fallback)
    int TABLE_HEIGHT;              // px
    bool ENABLE_UNDISTORTION;
};

class PerceptionNode : public rclcpp::Node {
public:
    explicit PerceptionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~PerceptionNode() override;

private:
    void image_callback(const air_hockey_robot_msgs::msg::PuckState::SharedPtr msg);
    void processing_loop();
    void load_parameters();

    rclcpp::Subscription<air_hockey_robot_msgs::msg::PuckState>::SharedPtr stream_sub_;
    rclcpp::Publisher<air_hockey_robot_msgs::msg::PuckDetection>::SharedPtr puck_pub_;

    cv::RotatedRect detectTable(cv::Mat& image);
    cv::Point2f detectPuck(const cv::Mat& grayImage);
    cv::Point2f imageToTableCoordinates(cv::Point2f imagePoint, int imageWidth, int imageHeight);

    bool loadCalibration(const std::string& filename = "calibration.yml");
    bool loadCachedPerspective(const std::string& filename = "table_perspective.yml");
    bool saveCachedPerspective(const std::string& filename = "table_perspective.yml");

    DetectorConfig config_;

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    cv::Rect tableBoundingRect_;
    cv::Size tableOutputSize_;
    cv::Mat tablePerspectiveMatrix_;   
    bool tableDetected_;
    bool tablePerspectiveCached_;

    std::thread processing_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<air_hockey_robot_msgs::msg::PuckState> frame_queue_;
    std::atomic<bool> is_running_{false};
};

} // namespace pc_node
