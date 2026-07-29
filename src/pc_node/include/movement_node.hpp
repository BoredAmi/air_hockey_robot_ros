#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/predicted_entry.hpp>
#include <std_msgs/msg/float32.hpp>

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
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_angle_pub_;
    rclcpp::TimerBase::SharedPtr angle_pub_timer_;

    double min_speed_for_robot_mm_s_;
    double defense_zone_buffer_mm_;
    uint64_t track_hold_duration_us_;

    cv::Point2f lastValidDefensePos_{-1.0f, -1.0f};
    bool trackTarget_ = false;
    uint64_t lastMoveTimeUs_ = 0;
};

} 
