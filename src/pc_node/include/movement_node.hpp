#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/predicted_entry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>

#include "config.hpp"
#include "movement.hpp"

namespace pc_node {

class MovementNode : public rclcpp::Node {
public:
    explicit MovementNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~MovementNode() override;

private:
    void entry_callback(const air_hockey_robot_msgs::msg::PredictedEntry::SharedPtr msg);

    Config load_config_parameters();
    void load_parameters();
    bool computeMovingTowardZone(int8_t zoneIndex, float vx, float vy) const;
    bool computePuckTooCloseToZone(int8_t zoneIndex, float puckX, float puckY, bool alreadyInZone) const;
    bool computePuckBehindZoneEntrance(int8_t zoneIndex, float puckX, float puckY) const;

    Config config_;
    MovementController mover_;

    rclcpp::Subscription<air_hockey_robot_msgs::msg::PredictedEntry>::SharedPtr entry_sub_;
    // [x, y] in table mm, published for visualization/telemetry.
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sent_position_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr actual_position_pub_;
    // Detection-to-send latency in ms - see getDetectionToSendLatencyMs()'s clock-sync caveat.
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr latency_pub_;
    rclcpp::TimerBase::SharedPtr position_pub_timer_;

    // True when a given /puck/predicted_entry is actually being acted on (not
    // rejected by the behind-zone/too-close/too-late-to-react gates) - lets
    // visualizers reflect the real decision instead of just msg->valid.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr target_accepted_pub_;

    double min_speed_for_robot_mm_s_;
    double defense_zone_buffer_mm_;
    uint64_t track_hold_duration_us_;
    double min_time_to_entry_s_;

    cv::Point2f lastValidDefensePos_{-1.0f, -1.0f};
    bool trackTarget_ = false;
    uint64_t lastMoveTimeUs_ = 0;
};

} 
