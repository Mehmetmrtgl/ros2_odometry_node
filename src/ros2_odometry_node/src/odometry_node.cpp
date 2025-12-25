#include "core/odometry_node.hpp"
#include "mecanum/mecanum_impl.hpp"
#include "ackermann/ackermann_impl.hpp"

OdometryNode::OdometryNode() : Node("ros2_odometry_node") {
    
    this->declare_parameter("drive_type", "MECANUM");
    std::string type = this->get_parameter("drive_type").as_string();

    if (type == "MECANUM") {
        vehicle_impl_ = std::make_unique<MecanumImpl>(this);
    } 
    else if (type == "ACKERMANN") { 

    }
    else {
        RCLCPP_ERROR(this->get_logger(), "unknow: %s", type.c_str());
        return;
    }

    if (vehicle_impl_) {
        vehicle_impl_->setup();
    }
}