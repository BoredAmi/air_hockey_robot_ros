#include "trajectory_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pc_node::TrajectoryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}