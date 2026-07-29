#include "trajectory_node.hpp"

namespace pc_node {

TrajectoryNode::TrajectoryNode(const rclcpp::NodeOptions & options)
: Node("trajectory_node", options),
    predictor_(nullptr)
{
        // load config from file first so defaults are consistent with other tools
        config_.loadFromFile();
        load_parameters();

        // construct predictor after config is loaded
        predictor_ = std::make_unique<TrajectoryPredictor>(config_);

    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    detection_sub_ = this->create_subscription<air_hockey_robot_msgs::msg::PuckDetection>(
        "/puck/detection",
        sub_qos,
        std::bind(&TrajectoryNode::detection_callback, this, std::placeholders::_1));

    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    entry_pub_ = this->create_publisher<air_hockey_robot_msgs::msg::PredictedEntry>(
        "/puck/predicted_entry", pub_qos);
    zone_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/puck/defense_zone", pub_qos);

    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&TrajectoryNode::on_parameter_change, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
        "TrajectoryNode wystartowany. /puck/detection -> /puck/predicted_entry");

    // Publish initial defense zone for debugger/visualizers
    publishDefenseZone();
}

void TrajectoryNode::load_parameters() {
    // Declare parameters using values loaded from config file as defaults
    this->declare_parameter<double>("physical_table_width", config_.PHYSICAL_TABLE_WIDTH);
    this->declare_parameter<double>("physical_table_height", config_.PHYSICAL_TABLE_HEIGHT);
    this->declare_parameter<double>("defense_zone_height", config_.DEFENSE_ZONE_HEIGHT);
    this->declare_parameter<double>("defense_zone_width", config_.DEFENSE_ZONE_WIDTH);
    this->declare_parameter<int>("where_defense_zone", config_.WHERE_DEFENSE_ZONE);

    // Read (or override) parameters into config
    config_.PHYSICAL_TABLE_WIDTH = this->get_parameter("physical_table_width").as_double();
    config_.PHYSICAL_TABLE_HEIGHT = this->get_parameter("physical_table_height").as_double();
    config_.DEFENSE_ZONE_HEIGHT = this->get_parameter("defense_zone_height").as_double();
    config_.DEFENSE_ZONE_WIDTH = this->get_parameter("defense_zone_width").as_double();
    config_.WHERE_DEFENSE_ZONE = this->get_parameter("where_defense_zone").as_int();

    if (predictor_) predictor_->setDefenseZone(config_.WHERE_DEFENSE_ZONE);
}

rcl_interfaces::msg::SetParametersResult TrajectoryNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters)
{
    bool zone_geometry_changed = false;

    for (const auto & param : parameters) {
        if (param.get_name() == "physical_table_width") {
            config_.PHYSICAL_TABLE_WIDTH = param.as_double();
            zone_geometry_changed = true;
        } else if (param.get_name() == "physical_table_height") {
            config_.PHYSICAL_TABLE_HEIGHT = param.as_double();
            zone_geometry_changed = true;
        } else if (param.get_name() == "defense_zone_height") {
            config_.DEFENSE_ZONE_HEIGHT = param.as_double();
            zone_geometry_changed = true;
        } else if (param.get_name() == "defense_zone_width") {
            config_.DEFENSE_ZONE_WIDTH = param.as_double();
            zone_geometry_changed = true;
        } else if (param.get_name() == "where_defense_zone") {
            config_.WHERE_DEFENSE_ZONE = param.as_int();
            zone_geometry_changed = true;
        }
    }

    if (zone_geometry_changed) {
        if (predictor_) predictor_->setDefenseZone(config_.WHERE_DEFENSE_ZONE);
        publishDefenseZone();
    }

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
}

void TrajectoryNode::detection_callback(const air_hockey_robot_msgs::msg::PuckDetection::SharedPtr msg) {
    air_hockey_robot_msgs::msg::PredictedEntry out;
    out.timestamp = msg->timestamp;
    out.valid = false;
    out.x = 0.0f;
    out.y = 0.0f;
    out.confidence = 0.0f;
    out.time_to_entry = -1.0f;

    if (!msg->is_detected) {
        entry_pub_->publish(out);
        return;
    }

    PuckPosition puckPos;
    puckPos.position = cv::Point2f(msg->x, msg->y);
    puckPos.timestamp = msg->timestamp;
    if (predictor_) {
        predictor_->addMeasurement(puckPos);

        cv::Point2f smoothedPos = predictor_->getCurrentPosition();
        cv::Point2f velocity = predictor_->getVelocity();
        out.puck_x = smoothedPos.x;
        out.puck_y = smoothedPos.y;
        out.vx = velocity.x;
        out.vy = velocity.y;
        out.puck_in_defense_zone = predictor_->isInDefenseZone(puckPos.position);
        out.defense_zone_index = static_cast<int8_t>(config_.WHERE_DEFENSE_ZONE);

        if (!out.puck_in_defense_zone) {
            double entryTimeSec = -1.0;
            cv::Point2f predicted = predictor_->predictEntryToDefenseZone(msg->timestamp, &entryTimeSec);
            if (predicted.x >= 0 && predicted.y >= 0) {
                out.valid = true;
                out.x = predicted.x;
                out.y = predicted.y;
                out.confidence = static_cast<float>(predictor_->getVelocityConfidence());
                out.time_to_entry = static_cast<float>(entryTimeSec);
            }
        }
    }

    entry_pub_->publish(out);
    // republish zone with each prediction so visualizers can stay in sync
    publishDefenseZone();
}

void TrajectoryNode::publishDefenseZone() {
    if (!predictor_ || !zone_pub_) return;
    std_msgs::msg::Float32MultiArray msg;
    msg.data.reserve(5);
    msg.data.push_back(static_cast<float>(predictor_->getDefenseZoneXMin()));
    msg.data.push_back(static_cast<float>(predictor_->getDefenseZoneXMax()));
    msg.data.push_back(static_cast<float>(predictor_->getDefenseZoneYMin()));
    msg.data.push_back(static_cast<float>(predictor_->getDefenseZoneYMax()));
    msg.data.push_back(static_cast<float>(config_.WHERE_DEFENSE_ZONE));
    zone_pub_->publish(msg);
}

} // namespace pc_node