#ifndef ACKERMANN_IMPL_HPP_
#define ACKERMANN_IMPL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "ros2_odometry_node/vehicle_interface.hpp"
#include "ackermann/ackermann_kinematics.hpp"
#include "ackermann/ackermann_data.hpp"
#include "ros2_odometry_node/robot_state.hpp"

class AckermannImpl : public VehicleInterface {
public:

    explicit AckermannImpl(rclcpp::Node* node);

    void setup() override;

private:
    rclcpp::Node* node_;
    
    AckermannKinematics kinematics_;
    
    AckermannStateData data_;

    rclcpp::Time last_time_;
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_left_enc_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_right_enc_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;


    void cb_left_enc(const std_msgs::msg::Float32::SharedPtr msg);
    void cb_right_enc(const std_msgs::msg::Float32::SharedPtr msg);

    void process_and_publish();
    
    void publish_state(const RobotState& state);
};

#endif // ACKERMANN_IMPL_HPP_