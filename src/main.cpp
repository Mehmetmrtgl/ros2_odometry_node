#include "rclcpp/rclcpp.hpp"
#include "core/odometry_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), ">>> Main: Program started.");
    auto node = std::make_shared<OdometryNode>();

    RCLCPP_INFO(node->get_logger(), ">>> Main: Node created, starting spin.");    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}