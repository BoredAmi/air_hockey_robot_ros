#include "movement_node.hpp"
#include <cmath>
#include <chrono>

namespace pc_node {

MovementNode::MovementNode(const rclcpp::NodeOptions & options)
: Node("movement_node", options),
  config_(load_config_parameters()),
  mover_(config_)
{
    load_parameters();

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    entry_sub_ = this->create_subscription<air_hockey_robot_msgs::msg::PredictedEntry>(
        "/puck/predicted_entry",
        sub_qos,
        std::bind(&MovementNode::entry_callback, this, std::placeholders::_1));

    sent_position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/robot/sent_position", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    actual_position_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/robot/actual_position", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    latency_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/detection_to_send_latency_ms", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    target_accepted_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "/robot/target_accepted", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    position_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            // Layout: [tableX, tableY, robotX, robotY]
            cv::Point2f sentTable = mover_.getSentPosition();
            cv::Point2f sentRobot = mover_.getSentPositionRobotFrame();
            std_msgs::msg::Float32MultiArray sent_msg;
            sent_msg.data = {sentTable.x, sentTable.y, sentRobot.x, sentRobot.y};
            sent_position_pub_->publish(sent_msg);

            cv::Point2f actualTable = mover_.getActualPosition();
            cv::Point2f actualRobot = mover_.getActualPositionRobotFrame();
            std_msgs::msg::Float32MultiArray actual_msg;
            actual_msg.data = {actualTable.x, actualTable.y, actualRobot.x, actualRobot.y};
            actual_position_pub_->publish(actual_msg);

            std_msgs::msg::Float32 latency_msg;
            latency_msg.data = mover_.getDetectionToSendLatencyMs();
            latency_pub_->publish(latency_msg);
        });

    RCLCPP_INFO(this->get_logger(),
        "MovementNode started. Listening: /puck/predicted_entry -> EGM/UDP to robot");
}

MovementNode::~MovementNode() {
    mover_.stop();
}

Config MovementNode::load_config_parameters() {
    // Defaults come from config.json so every node agrees on table/zone geometry -
    // ROS parameters can still override them at launch if needed.
    Config fileConfig;
    fileConfig.loadFromFile();

    this->declare_parameter<double>("physical_table_width", fileConfig.PHYSICAL_TABLE_WIDTH);
    this->declare_parameter<double>("physical_table_height", fileConfig.PHYSICAL_TABLE_HEIGHT);
    this->declare_parameter<double>("defense_zone_width", fileConfig.DEFENSE_ZONE_WIDTH);
    this->declare_parameter<double>("defense_zone_height", fileConfig.DEFENSE_ZONE_HEIGHT);
    this->declare_parameter<int>("robot_origin_corner", fileConfig.robot_origin_corner);

    fileConfig.PHYSICAL_TABLE_WIDTH = this->get_parameter("physical_table_width").as_double();
    fileConfig.PHYSICAL_TABLE_HEIGHT = this->get_parameter("physical_table_height").as_double();
    fileConfig.DEFENSE_ZONE_WIDTH = this->get_parameter("defense_zone_width").as_double();
    fileConfig.DEFENSE_ZONE_HEIGHT = this->get_parameter("defense_zone_height").as_double();
    fileConfig.robot_origin_corner = this->get_parameter("robot_origin_corner").as_int();

    return fileConfig;
}

void MovementNode::load_parameters() {
    this->declare_parameter<double>("min_speed_for_robot_mm_s", 100.0);
    this->declare_parameter<double>("defense_zone_buffer_mm", 450.0);
    this->declare_parameter<int>("track_hold_duration_ms", 1000);
    this->declare_parameter<double>("min_time_to_entry_s", 0.15);

    min_speed_for_robot_mm_s_ = this->get_parameter("min_speed_for_robot_mm_s").as_double();
    defense_zone_buffer_mm_ = this->get_parameter("defense_zone_buffer_mm").as_double();
    track_hold_duration_us_ =
        static_cast<uint64_t>(this->get_parameter("track_hold_duration_ms").as_int()) * 1000ULL;
    min_time_to_entry_s_ = this->get_parameter("min_time_to_entry_s").as_double();
}


bool MovementNode::computeMovingTowardZone(int8_t zoneIndex, float vx, float vy) const {
    switch (zoneIndex) {
        case 0: return vx <= 0;  
        case 1: return vx >= 0;  
        case 2: return vy <= 0;  
        case 3: return vy >= 0;  
        default: return true;
    }
}


bool MovementNode::computePuckTooCloseToZone(int8_t zoneIndex, float puckX, float puckY, bool alreadyInZone) const {
    if (alreadyInZone) return true;
    switch (zoneIndex) {
        case 0: return puckY < config_.DEFENSE_ZONE_HEIGHT + defense_zone_buffer_mm_;
        case 1: return puckY > config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_HEIGHT - defense_zone_buffer_mm_;
        case 2: return puckX < config_.DEFENSE_ZONE_WIDTH + defense_zone_buffer_mm_;
        case 3: return puckX > config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH - defense_zone_buffer_mm_;
        default: return false;
    }
}

bool MovementNode::computePuckBehindZoneEntrance(int8_t zoneIndex, float puckX, float puckY) const {
    switch (zoneIndex) {
        case 0: return puckY <= config_.DEFENSE_ZONE_HEIGHT;                                    // top
        case 1: return puckY >= config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_HEIGHT;    // bottom
        case 2: return puckX <= config_.DEFENSE_ZONE_WIDTH;                                     // left
        case 3: return puckX >= config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH;      // right
        default: return false;
    }
}

void MovementNode::entry_callback(const air_hockey_robot_msgs::msg::PredictedEntry::SharedPtr msg) {
    cv::Point2f targetTablePos(-1.0f, -1.0f);

    uint64_t pcReceiveTimeUs = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    mover_.updatePuckPosition(cv::Point2f(msg->puck_x, msg->puck_y), cv::Point2f(msg->vx, msg->vy),
                               msg->confidence, msg->time_to_entry, pcReceiveTimeUs);

    bool puckBehindZone = computePuckBehindZoneEntrance(msg->defense_zone_index, msg->puck_x, msg->puck_y);
    bool puckTooCloseToZone = computePuckTooCloseToZone(
        msg->defense_zone_index, msg->puck_x, msg->puck_y, msg->puck_in_defense_zone);

    bool targetAccepted = !puckBehindZone && !puckTooCloseToZone && msg->valid;
    std_msgs::msg::Bool accepted_msg;
    accepted_msg.data = targetAccepted;
    target_accepted_pub_->publish(accepted_msg);

    if (targetAccepted) {
        /*
        double speed = std::hypot(msg->vx, msg->vy);

        bool movingTowardZone = computeMovingTowardZone(msg->defense_zone_index, msg->vx, msg->vy);
        bool puckTooCloseToZone = computePuckTooCloseToZone(
            msg->defense_zone_index, msg->puck_x, msg->puck_y, msg->puck_in_defense_zone);

        bool dynamicAttackPossible =
            movingTowardZone && (speed >= min_speed_for_robot_mm_s_) && !puckTooCloseToZone;

        if (dynamicAttackPossible) {
            targetTablePos = cv::Point2f(msg->x, msg->y);
            lastValidDefensePos_ = targetTablePos;
            trackTarget_ = true;
            lastMoveTimeUs_ = msg->timestamp;

            RCLCPP_INFO(this->get_logger(),
                "EGM MOVE: x=%.1f y=%.1f speed=%.1f mm/s conf=%.2f",
                targetTablePos.x, targetTablePos.y, speed, msg->confidence);
        } else if (trackTarget_ && (msg->timestamp - lastMoveTimeUs_) < track_hold_duration_us_) {

            targetTablePos = lastValidDefensePos_;
        } else {
            trackTarget_ = false;
            targetTablePos = cv::Point2f(-1.0f, -1.0f);
        }
        */
       cv::Point2f newTarget(msg->x, msg->y);
       bool haveExistingTarget = trackTarget_ && lastValidDefensePos_.x >= 0.0f;
       if (!haveExistingTarget || cv::norm(newTarget - lastValidDefensePos_) > TARGET_DEADBAND_MM) {
           lastValidDefensePos_ = newTarget;
       }
       targetTablePos = lastValidDefensePos_;
       trackTarget_ = true;
       lastMoveTimeUs_ = msg->timestamp;
    } else {

        if (trackTarget_ && (msg->timestamp - lastMoveTimeUs_) < track_hold_duration_us_) {
            targetTablePos = lastValidDefensePos_;
        } else {
            trackTarget_ = false;
        }
    }


    mover_.moveTo(targetTablePos, lastMoveTimeUs_);
}

} // namespace pc_node