#include "ros2_odometry_node/odometry_node.hpp"
#include "ros2_odometry_node/mecanum_impl.hpp"
#include "ros2_odometry_node/ackermann_impl.hpp" // <--- YENİ EKLEME

OdometryNode::OdometryNode() : Node("ros2_odometry_node") {
    
    this->declare_parameter("drive_type", "MECANUM");
    std::string type = this->get_parameter("drive_type").as_string();

    if (type == "MECANUM") {
        vehicle_impl_ = std::make_unique<MecanumImpl>(this);
    } 
    else if (type == "ACKERMANN") { // <--- YENİ BLOK

    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Bilinmeyen tip: %s", type.c_str());
        return;
    }

    if (vehicle_impl_) {
        vehicle_impl_->setup();
    }
}