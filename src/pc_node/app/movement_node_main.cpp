#include "movement_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pc_node::MovementNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
