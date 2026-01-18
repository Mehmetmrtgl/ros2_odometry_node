#ifndef MECANUM_IMPL_HPP_
#define MECANUM_IMPL_HPP_

#include "ros2_odometry_node/vehicle_interface.hpp"
#include "mecanum/mecanum_kinematics.hpp"
#include "mecanum/mecanum_data.hpp"
#include "ros2_odometry_node/robot_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

class MecanumImpl : public VehicleInterface {
public:
    explicit MecanumImpl(rclcpp::Node* node);

    void setup() override;

private:
    rclcpp::Node* node_; 
    
    MecanumKinematics kinematics_;
    MecanumWheelStates data_;

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_enc_group1_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_enc_group2_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vel_group1_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_vel_group2_;

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheel_pos_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;

    void cb_enc_group1(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void cb_enc_group2(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void cb_vel_group1(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
    void cb_vel_group2(const std_msgs::msg::Float32MultiArray::SharedPtr msg);

    void process_and_publish();
    void publish_state(const RobotState& state);
};

#endif // MECANUM_IMPL_HPP_