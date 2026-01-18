#include "ackermann/ackermann_impl.hpp"
#include "utils/odometry_parameters.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h" 
#include "nav_msgs/msg/path.hpp" 

using std::placeholders::_1;

AckermannImpl::AckermannImpl(rclcpp::Node* node) : node_(node) {
    data_.left_ticks = 0;
    data_.right_ticks = 0;
    first_measurement_ = true;
    
    // TF Broadcaster nesnesini oluşturuyoruz
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
}

void AckermannImpl::setup() {
    RCLCPP_INFO(node_->get_logger(), "ACKERMANN Mode (KAIST irp_sen_msgs) started...");

    OdometryParameters param_handler(node_);
    AckermannParameters params = param_handler.getAckermannParams();
    kinematics_.setConfig(params);
    
    // Publishers
    pub_odom_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    pub_path_ = node_->create_publisher<nav_msgs::msg::Path>("/path", 10);
    
    RCLCPP_INFO(node_->get_logger(), "Publishers created: /odom and /path");

    // Path mesajının sabit bilgilerini ayarla
    path_msg_.header.frame_id = "odom";

    sub_encoder_ = node_->create_subscription<irp_sen_msgs::msg::Encoder>(
        "/encoder_count", 10, std::bind(&AckermannImpl::cb_encoder, this, _1));
        
    RCLCPP_INFO(node_->get_logger(), "Listening: /encoder_count");
}

void AckermannImpl::cb_encoder(const irp_sen_msgs::msg::Encoder::SharedPtr msg) {
    data_.left_ticks = msg->left_count;
    data_.right_ticks = msg->right_count;

    rclcpp::Time msg_time(msg->header.stamp);
    
    RCLCPP_INFO(node_->get_logger(), 
                "Message Time -> Sec: %ld, NanoSec: %ld", 
                static_cast<long>(msg_time.seconds()), 
                static_cast<long>(msg_time.nanoseconds() % 1000000000));

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
    // 1. Odometry Mesajını Oluştur ve Yayınla
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

    pub_odom_->publish(odom_msg);

    // 2. TF (Transform) Mesajını Yayınla
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = state.pose.x;
    t.transform.translation.y = state.pose.y;
    t.transform.translation.z = 0.0;
    t.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_->sendTransform(t);

    // 3. Path Mesajını Güncelle ve Yayınla
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.stamp = stamp;
    pose_stamped.header.frame_id = "odom";
    pose_stamped.pose = odom_msg.pose.pose; // Pose verisini kopyala

    path_msg_.header.stamp = stamp;
    path_msg_.poses.push_back(pose_stamped);

    pub_path_->publish(path_msg_);
}