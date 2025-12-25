#include "ackermann/ackermann_impl.hpp"
#include "utils/odometry_parameters.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

AckermannImpl::AckermannImpl(rclcpp::Node* node) : node_(node) {
    data_.left_ticks = 0.0;
    data_.right_ticks = 0.0;
}

void AckermannImpl::setup() {
    
    OdometryParameters param_handler(node_);
    AckermannParameters params = param_handler.getAckermannParams();
    kinematics_.setConfig(params);

    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    sub_left_enc_ = node_->create_subscription<std_msgs::msg::Float32>(
        "left_encoder_ticks", 10, std::bind(&AckermannImpl::cb_left_enc, this, _1));
        
    sub_right_enc_ = node_->create_subscription<std_msgs::msg::Float32>(
        "right_encoder_ticks", 10, std::bind(&AckermannImpl::cb_right_enc, this, _1));
    
    last_time_ = node_->get_clock()->now();
}

void AckermannImpl::cb_left_enc(const std_msgs::msg::Float32::SharedPtr msg) {
    data_.left_ticks = msg->data;
}

void AckermannImpl::cb_right_enc(const std_msgs::msg::Float32::SharedPtr msg) {
    data_.right_ticks = msg->data;
    process_and_publish();
}

void AckermannImpl::process_and_publish() {

    rclcpp::Time now = node_->get_clock()->now();
    double dt = (now - last_time_).seconds();
    
    last_time_ = now;

    kinematics_.update(data_, dt);
    
    RobotState state = kinematics_.getState();
    publish_state(state);
}


void AckermannImpl::publish_state(const RobotState& state) {
    auto now = node_->get_clock()->now();

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
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.angular.z = state.velocity.omega; 

    pub_odom_->publish(odom_msg);
}