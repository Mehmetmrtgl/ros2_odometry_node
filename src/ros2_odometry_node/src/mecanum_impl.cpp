#include "ros2_odometry_node/mecanum_impl.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>

using std::placeholders::_1;

MecanumImpl::MecanumImpl(rclcpp::Node* node) : node_(node) {
    data_.ticks.fill(0.0f);
    data_.rpm.fill(0.0f);
}

void MecanumImpl::setup() {
    RCLCPP_INFO(node_->get_logger(), "Mecanum Implementasyonu Yukleniyor...");

    if (!node_->has_parameter("ticks_per_rev")) node_->declare_parameter("ticks_per_rev", 4200.0);
    if (!node_->has_parameter("wheel_radius"))  node_->declare_parameter("wheel_radius", 0.04);
    if (!node_->has_parameter("robot_l"))       node_->declare_parameter("robot_l", 0.20);
    if (!node_->has_parameter("robot_w"))       node_->declare_parameter("robot_w", 0.23);
    if (!node_->has_parameter("angle_epsilon")) node_->declare_parameter("angle_epsilon", 0.0001);

    MecanumParameters params;
    params.ticks_per_rev = node_->get_parameter("ticks_per_rev").as_double();
    params.wheel_radius  = node_->get_parameter("wheel_radius").as_double();
    params.robot_l       = node_->get_parameter("robot_l").as_double();
    params.robot_w       = node_->get_parameter("robot_w").as_double();
    params.angle_epsilon = node_->get_parameter("angle_epsilon").as_double();

    kinematics_.setConfig(params);
    
    RCLCPP_INFO(node_->get_logger(), "Mecanum Ayarlari: R=%.3f, L=%.3f, W=%.3f", 
                params.wheel_radius, params.robot_l, params.robot_w);

    pub_wheel_pos_ = node_->create_publisher<std_msgs::msg::Float32MultiArray>("wheel_positions", 10);
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    sub_enc_group1_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "encoder1", 10, std::bind(&MecanumImpl::cb_enc_group1, this, _1));
    
    sub_enc_group2_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "encoder2", 10, std::bind(&MecanumImpl::cb_enc_group2, this, _1));

    sub_vel_group1_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "velocity1", 10, std::bind(&MecanumImpl::cb_vel_group1, this, _1));
    
    sub_vel_group2_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>(
        "velocity2", 10, std::bind(&MecanumImpl::cb_vel_group2, this, _1));
}

void MecanumImpl::cb_enc_group1(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        data_.ticks[2] = msg->data[0]; 
        data_.ticks[3] = msg->data[1]; 
    }
}

void MecanumImpl::cb_enc_group2(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        data_.ticks[0] = msg->data[0]; 
        data_.ticks[1] = msg->data[1]; 
    }
}

void MecanumImpl::cb_vel_group1(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        data_.rpm[0] = msg->data[0]; 
        data_.rpm[1] = msg->data[1]; 
    }
}

void MecanumImpl::cb_vel_group2(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    if (msg->data.size() >= 2) {
        data_.rpm[2] = msg->data[0]; 
        data_.rpm[3] = msg->data[1]; 

        process_and_publish();
    }
}

void MecanumImpl::process_and_publish() {
    kinematics_.update(data_);

    RobotState state = kinematics_.getState();

    publish_state(state);
}

void MecanumImpl::publish_state(const RobotState& state) {
    auto now = node_->get_clock()->now();

    std_msgs::msg::Float32MultiArray wheel_msg;
    wheel_msg.data = {
        state.pose.x, 
        state.pose.y, 
        state.pose.theta, 
        (float)(state.pose.theta * 180.0 / M_PI)
    };
    pub_wheel_pos_->publish(wheel_msg);

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = state.pose.x;
    odom_msg.pose.pose.position.y = state.pose.y;
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.pose.theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    odom_msg.twist.twist.linear.x = state.velocity.vx;
    odom_msg.twist.twist.linear.y = state.velocity.vy;
    odom_msg.twist.twist.angular.z = state.velocity.omega;

    pub_odom_->publish(odom_msg);
}