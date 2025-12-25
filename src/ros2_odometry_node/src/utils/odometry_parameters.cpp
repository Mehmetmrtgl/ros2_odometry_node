#include "utils/odometry_parameters.hpp"

OdometryParameters::OdometryParameters(rclcpp::Node* node) : node_(node) {

    node_->declare_parameter("drive_type", "MECANUM"); 
    
    node_->declare_parameter("ticks_per_rev", 4200.0);
    node_->declare_parameter("wheel_radius", 0.04);
    node_->declare_parameter("robot_l", 0.20);
    node_->declare_parameter("robot_w", 0.23);
    node_->declare_parameter("angle_epsilon", 0.0001);
}

DriveType OdometryParameters::getDriveType() {
    std::string type_str = node_->get_parameter("drive_type").as_string();

    if (type_str == "MECANUM") {
        RCLCPP_INFO(node_->get_logger(), "Parametre Okundu: Surus Tipi -> MECANUM");
        return DriveType::MECANUM;
    } 
    else if (type_str == "DIFFERENTIAL") {
        return DriveType::DIFFERENTIAL;
    } 
    else if (type_str == "ACKERMANN") {
        return DriveType::ACKERMANN;
    } 
    else {
        RCLCPP_ERROR(node_->get_logger(), "Gecersiz Arac Tipi: %s", type_str.c_str());
        return DriveType::UNKNOWN;
    }
}

MecanumParameters OdometryParameters::getMecanumParams() {
    MecanumParameters params;
    
    params.ticks_per_rev = node_->get_parameter("ticks_per_rev").as_double();
    params.wheel_radius  = node_->get_parameter("wheel_radius").as_double();
    params.robot_l       = node_->get_parameter("robot_l").as_double();
    params.robot_w       = node_->get_parameter("robot_w").as_double();
    params.angle_epsilon = node_->get_parameter("angle_epsilon").as_double();

    RCLCPP_INFO(node_->get_logger(), "Mecanum Parametreleri Yuklendi: R=%.3f, L=%.3f, W=%.3f", 
                params.wheel_radius, params.robot_l, params.robot_w);

    return params;
}