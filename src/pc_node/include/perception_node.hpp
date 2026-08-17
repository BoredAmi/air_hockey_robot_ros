#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_state.hpp>
#include <air_hockey_robot_msgs/msg/puck_detection.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include "table_marker_layout.hpp"

#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <map>

namespace pc_node {

struct DetectorConfig {
    int PUCK_ARUCO_ID;
    double PHYSICAL_TABLE_WIDTH;   // mm
    double PHYSICAL_TABLE_HEIGHT;  // mm
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

    bool loadTableMarkerLayout(const std::string& filename = "table_markers.yml");

    bool loadCalibration(const std::string& filename = "calibration.yml");

    DetectorConfig config_;

    // Puck is identified by an AprilTag 16h5 marker rather than a color/shape blob. small change but more reliable in our shitty garage
    cv::Ptr<cv::aruco::Dictionary> arucoDict_;
    cv::Ptr<cv::aruco::DetectorParameters> arucoParams_;

    cv::Mat cameraMatrix_;
    cv::Mat distCoeffs_;

    std::map<int, cv::Point2f> tableMarkerLayout_;  // marker id -> table-frame mm position
    cv::Mat tableHomography_;                       // undistorted-pixel -> table mm
    bool haveTableHomography_ = false;

    std::thread processing_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::queue<air_hockey_robot_msgs::msg::PuckState> frame_queue_;
    std::atomic<bool> is_running_{false};
};

} // namespace pc_node
