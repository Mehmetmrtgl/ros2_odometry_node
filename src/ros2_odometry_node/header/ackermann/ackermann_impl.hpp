#ifndef ACKERMANN_IMPL_HPP_
#define ACKERMANN_IMPL_HPP_

#include <memory>
#include <vector>

// ROS 2 Core
#include "rclcpp/rclcpp.hpp"

// ROS 2 Messages
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "irp_sen_msgs/msg/encoder.hpp"

// TF2
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// Local Headers
#include "ros2_odometry_node/vehicle_interface.hpp"
#include "ackermann/ackermann_kinematics.hpp"
#include "ackermann/ackermann_data.hpp"
#include "ros2_odometry_node/robot_state.hpp"

class AckermannImpl : public VehicleInterface {
public:
    /**
     * @brief Constructor for Ackermann Implementation
     * @param node Pointer to the ROS 2 node
     */
    explicit AckermannImpl(rclcpp::Node* node);

    /**
     * @brief Setup publishers, subscribers and parameters
     */
    void setup() override;

private:
    rclcpp::Node* node_;
    
    AckermannKinematics kinematics_;
    
    AckermannStateData data_;
    rclcpp::Time last_time_;
    bool first_measurement_; 

    rclcpp::Subscription<irp_sen_msgs::msg::Encoder>::SharedPtr sub_encoder_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    nav_msgs::msg::Path path_msg_;

    /**
     * @brief Callback for encoder messages
     */
    void cb_encoder(const irp_sen_msgs::msg::Encoder::SharedPtr msg);
    
    /**
     * @brief Processes kinematics and triggers publishing
     */
    void process_and_publish(double dt, rclcpp::Time stamp);

    /**
     * @brief Publishes Odom, TF, and Path messages
     */
    void publish_state(const RobotState& state, rclcpp::Time stamp);
};

#endif // ACKERMANN_IMPL_HPP_