#include "ackermann/ackermann_impl.hpp"
#include "utils/odometry_parameters.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;

AckermannImpl::AckermannImpl(rclcpp::Node* node) : node_(node) {
    data_.left_ticks = 0;
    data_.right_ticks = 0;
    first_measurement_ = true;
}

void AckermannImpl::setup() {
    RCLCPP_INFO(node_->get_logger(), "ACKERMANN Mode (KAIST irp_sen_msgs) started...");

    OdometryParameters param_handler(node_);
    AckermannParameters params = param_handler.getAckermannParams();
    kinematics_.setConfig(params);
    
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    RCLCPP_INFO(node_->get_logger(), "Publisher created");

    sub_encoder_ = node_->create_subscription<irp_sen_msgs::msg::Encoder>(
        "/encoder_count", 10, std::bind(&AckermannImpl::cb_encoder, this, _1));
        
    RCLCPP_INFO(node_->get_logger(), "Listening: /encoder_count");
}

void AckermannImpl::cb_encoder(const irp_sen_msgs::msg::Encoder::SharedPtr msg) {
    // Mesajdan gelen verileri doğrudan tam sayı olarak sakla
    data_.left_ticks = msg->left_count;
    data_.right_ticks = msg->right_count;

    rclcpp::Time msg_time(msg->header.stamp);
    
    // Zaman bilgisini tam sayı olarak bas (Casting double to long)
    RCLCPP_INFO(node_->get_logger(), 
                "Message Time -> Sec: %ld, NanoSec: %ld", 
                static_cast<long>(msg_time.seconds()), 
                static_cast<long>(msg_time.nanoseconds() % 1000000000));

    // Tick değerlerini tam sayı olarak bas (%ld: long int)
    RCLCPP_INFO(node_->get_logger(), 
                "Ticks -> L: %ld, R: %ld", 
                data_.left_ticks, 
                data_.right_ticks);

    double dt = 0.01; 
    if (first_measurement_) {
        last_time_ = msg_time;
        first_measurement_ = false;
        return; 
    } else {
        dt = (msg_time - last_time_).seconds();
        last_time_ = msg_time;
    }

    if (dt <= 0.000001) return;

    process_and_publish(dt, msg_time);
}

void AckermannImpl::process_and_publish(double dt, rclcpp::Time stamp) {

    kinematics_.update(data_, dt);
    

    RobotState state = kinematics_.getState();
    publish_state(state, stamp);
}

void AckermannImpl::publish_state(const RobotState& state, rclcpp::Time stamp) {

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = stamp; 
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";


    odom_msg.pose.pose.position.x = state.pose.x;
    odom_msg.pose.pose.position.y = state.pose.y;
    odom_msg.pose.pose.position.z = 0.0;


    tf2::Quaternion q;
    q.setRPY(0, 0, state.pose.theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);


    odom_msg.twist.twist.linear.x = state.velocity.vx;
    odom_msg.twist.twist.angular.z = state.velocity.omega;

    // RCLCPP_INFO(node_->get_logger(), "Publishing /odom at x: %.2f, y: %.2f", state.pose.x, state.pose.y);

    pub_odom_->publish(odom_msg);
}