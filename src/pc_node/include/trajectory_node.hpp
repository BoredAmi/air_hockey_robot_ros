#pragma once

#include <rclcpp/rclcpp.hpp>
#include <air_hockey_robot_msgs/msg/puck_detection.hpp>
#include <air_hockey_robot_msgs/msg/predicted_entry.hpp>
#include "trajectory.hpp"

#include "config.hpp"
#include <memory>
#include <std_msgs/msg/float32_multi_array.hpp>

namespace pc_node {

class TrajectoryNode : public rclcpp::Node {
public:
    explicit TrajectoryNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void detection_callback(const air_hockey_robot_msgs::msg::PuckDetection::SharedPtr msg);
    void load_parameters();
    rcl_interfaces::msg::SetParametersResult on_parameter_change(
        const std::vector<rclcpp::Parameter> & parameters);

    Config config_;
    std::unique_ptr<TrajectoryPredictor> predictor_;

    rclcpp::Subscription<air_hockey_robot_msgs::msg::PuckDetection>::SharedPtr detection_sub_;
    rclcpp::Publisher<air_hockey_robot_msgs::msg::PredictedEntry>::SharedPtr entry_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr zone_pub_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
    void publishDefenseZone();
};

} // namespace pc_node