#include "utils/odometry_parameters.hpp"

OdometryParameters::OdometryParameters(rclcpp::Node* node) : node_(node) {

    node_->declare_parameter("drive_type", "MECANUM");
    
    current_drive_type_ = node_->get_parameter("drive_type").as_string();

    RCLCPP_INFO(node_->get_logger(), "Secilen Surus Tipi: %s", current_drive_type_.c_str());

    if (current_drive_type_ == "MECANUM") {
        declareMecanum();
    } 
    else if (current_drive_type_ == "ACKERMANN") {
        declareAckermann();
    }
    else {
        RCLCPP_ERROR(node_->get_logger(), "Unknow type");
    }
}

void OdometryParameters::declareMecanum() {
    node_->declare_parameter("mecanum.ticks_per_rev", 4200.0);
    node_->declare_parameter("mecanum.wheel_radius", 0.04);
    node_->declare_parameter("mecanum.robot_l", 0.20);
    node_->declare_parameter("mecanum.robot_w", 0.23);
    node_->declare_parameter("mecanum.angle_epsilon", 0.0001);
    RCLCPP_INFO(node_->get_logger(), "-> Mecanum parameters have been introduced to the system.");
}

void OdometryParameters::declareAckermann() {
    node_->declare_parameter("ackermann.wheelbase", 1.52);
    node_->declare_parameter("ackermann.ticks_per_rev", 4096.0);
    node_->declare_parameter("ackermann.left_wheel_diameter", 0.62);
    node_->declare_parameter("ackermann.right_wheel_diameter", 0.62);

    RCLCPP_INFO(node_->get_logger(), "-> Ackermann parameters have been introduced to the system..");
}


DriveType OdometryParameters::getDriveType() {
    if (current_drive_type_ == "MECANUM") return DriveType::MECANUM;
    if (current_drive_type_ == "ACKERMANN") return DriveType::ACKERMANN;
    return DriveType::UNKNOWN;
}

MecanumParameters OdometryParameters::getMecanumParams() {
    MecanumParameters params;

    if (current_drive_type_ != "MECANUM") {
        RCLCPP_ERROR(node_->get_logger(), "ERROR");
        return params; 
    }

    params.ticks_per_rev = node_->get_parameter("mecanum.ticks_per_rev").as_double();
    params.wheel_radius  = node_->get_parameter("mecanum.wheel_radius").as_double();
    params.robot_l       = node_->get_parameter("mecanum.robot_l").as_double();
    params.robot_w       = node_->get_parameter("mecanum.robot_w").as_double();
    params.angle_epsilon = node_->get_parameter("mecanum.angle_epsilon").as_double();
    
    return params;
}

AckermannParameters OdometryParameters::getAckermannParams() {
    AckermannParameters params;
    
    if (current_drive_type_ != "ACKERMANN") {
        return params;
    }

    params.wheelbase     = node_->get_parameter("ackermann.wheelbase").as_double();
    params.ticks_per_rev = node_->get_parameter("ackermann.ticks_per_rev").as_double();
    
    params.left_wheel_diameter  = node_->get_parameter("ackermann.left_wheel_diameter").as_double();
    params.right_wheel_diameter = node_->get_parameter("ackermann.right_wheel_diameter").as_double();
 
    return params;
}