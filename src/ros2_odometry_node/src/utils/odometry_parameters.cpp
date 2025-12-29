#include "utils/odometry_parameters.hpp"

OdometryParameters::OdometryParameters(rclcpp::Node* node) : node_(node) {

    if (!node_->has_parameter("drive_type")) {
        node_->declare_parameter("drive_type", "MECANUM");
    }
    
    current_drive_type_ = node_->get_parameter("drive_type").as_string();
    RCLCPP_INFO(node_->get_logger(), "Secilen Surus Tipi: %s", current_drive_type_.c_str());

    if (current_drive_type_ == "MECANUM") {
        declareMecanum();
    } 
    else if (current_drive_type_ == "ACKERMANN") {
        declareAckermann();
    }
}

void OdometryParameters::declareMecanum() {
    node_->declare_parameter("mecanum.ticks_per_rev", 4200.0);
    node_->declare_parameter("mecanum.wheel_radius", 0.04);
    node_->declare_parameter("mecanum.robot_l", 0.20);
    node_->declare_parameter("mecanum.robot_w", 0.23);
    node_->declare_parameter("mecanum.angle_epsilon", 0.0001);

    // Parametreleri çek ve terminale yazdır
    double t_p_r = node_->get_parameter("mecanum.ticks_per_rev").as_double();
    double r = node_->get_parameter("mecanum.wheel_radius").as_double();
    double l = node_->get_parameter("mecanum.robot_l").as_double();
    double w = node_->get_parameter("mecanum.robot_w").as_double();
    double a_e = node_->get_parameter("mecanum.angle_epsilon").as_double();

    RCLCPP_INFO(node_->get_logger(), ">>> Mecanum Config: Ticks per rev: %.3f,  Wheel Radius: %.3f, L: %.3f, W: %.3f, angle_epsilon: %.3f", t_p_r, r, l, w, a_e);

}

void OdometryParameters::declareAckermann() {
    node_->declare_parameter("ackermann.wheelbase", 1.52);
    node_->declare_parameter("ackermann.ticks_per_rev", 4096.0);
    node_->declare_parameter("ackermann.left_wheel_diameter", 0.62);
    node_->declare_parameter("ackermann.right_wheel_diameter", 0.62);

    // Parametreleri çek ve terminale yazdır
    double wb = node_->get_parameter("ackermann.wheelbase").as_double();
    double t_p_r_a = node_->get_parameter("ackermann.ticks_per_rev").as_double();
    double lwd = node_->get_parameter("ackermann.left_wheel_diameter").as_double();
    double rwd = node_->get_parameter("ackermann.right_wheel_diameter").as_double();

    RCLCPP_INFO(node_->get_logger(), ">>> Ackermann Config: ticks_per_rev: %.3f, Wheelbase: %.3f, Left Wheel Dia: %.3f, Right Wheel Dia: %.3f", t_p_r_a, wb, lwd, rwd);

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