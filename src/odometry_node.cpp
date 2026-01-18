#include "core/odometry_node.hpp"
#include "mecanum/mecanum_impl.hpp"
#include "ackermann/ackermann_impl.hpp"

OdometryNode::OdometryNode() : Node("ros2_odometry_node", 
    rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)
) {
    std::string type = this->get_parameter("drive_type").as_string();

    RCLCPP_INFO(this->get_logger(), ">>> OdometryNode: Parameter read. Selected type: %s", type.c_str());

    if (type == "MECANUM") {
        RCLCPP_INFO(this->get_logger(), ">>> OdometryNode: Loading MecanumImpl...");
        vehicle_impl_ = std::make_unique<MecanumImpl>(this);
    } 
    else if (type == "ACKERMANN") { 
        RCLCPP_INFO(this->get_logger(), ">>> OdometryNode: Loading AckermannImpl...");
        vehicle_impl_ = std::make_unique<AckermannImpl>(this);
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Unknown drive type: %s", type.c_str());
    }

    if (vehicle_impl_) {
        vehicle_impl_->setup();
    }
}