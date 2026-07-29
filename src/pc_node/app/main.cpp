#include "perception_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<pc_node::PerceptionNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}