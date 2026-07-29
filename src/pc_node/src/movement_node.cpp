#include "movement_node.hpp"
#include <cmath>

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

    angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/sent_angle", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    actual_angle_pub_ = this->create_publisher<std_msgs::msg::Float32>(
        "/robot/actual_angle", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    angle_pub_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        [this]() {
            std_msgs::msg::Float32 sent_msg;
            sent_msg.data = mover_.getSentAngle();
            angle_pub_->publish(sent_msg);

            std_msgs::msg::Float32 actual_msg;
            actual_msg.data = mover_.getActualAngle();
            actual_angle_pub_->publish(actual_msg);
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
    this->declare_parameter<double>("defense_zone_buffer_mm", 100.0);
    this->declare_parameter<int>("track_hold_duration_ms", 1000);

    min_speed_for_robot_mm_s_ = this->get_parameter("min_speed_for_robot_mm_s").as_double();
    defense_zone_buffer_mm_ = this->get_parameter("defense_zone_buffer_mm").as_double();
    track_hold_duration_us_ =
        static_cast<uint64_t>(this->get_parameter("track_hold_duration_ms").as_int()) * 1000ULL;
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
        case 0: return puckX < config_.DEFENSE_ZONE_WIDTH + defense_zone_buffer_mm_;
        case 1: return puckX > config_.PHYSICAL_TABLE_WIDTH - config_.DEFENSE_ZONE_WIDTH - defense_zone_buffer_mm_;
        case 2: return puckY < config_.DEFENSE_ZONE_HEIGHT + defense_zone_buffer_mm_;
        case 3: return puckY > config_.PHYSICAL_TABLE_HEIGHT - config_.DEFENSE_ZONE_HEIGHT - defense_zone_buffer_mm_;
        default: return false;
    }
}

bool MovementNode::computePuckBehindZoneEntrance(int8_t zoneIndex, float puckX, float puckY) const {
    // "Behind" = puck has already crossed the zone's entrance line (the boundary
    // facing the open table), so it's out of the robot's reach - ignore it entirely
    // rather than chasing something already past the defense line.
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

    bool puckBehindZone = computePuckBehindZoneEntrance(msg->defense_zone_index, msg->puck_x, msg->puck_y);

    if (!puckBehindZone && msg->valid) {
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
       targetTablePos = cv::Point2f(msg->x, msg->y);
       lastValidDefensePos_ = targetTablePos;
       trackTarget_ = true;
       lastMoveTimeUs_ = msg->timestamp;
    } else {

        if (trackTarget_ && (msg->timestamp - lastMoveTimeUs_) < track_hold_duration_us_) {
            targetTablePos = lastValidDefensePos_;
        } else {
            trackTarget_ = false;
        }
    }


    mover_.moveTo(targetTablePos);
}

} // namespace pc_node