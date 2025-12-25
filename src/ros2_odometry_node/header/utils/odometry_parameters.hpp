#ifndef ODOMETRY_PARAMETERS_HPP_
#define ODOMETRY_PARAMETERS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_odometry_node/enums.hpp"
#include "mecanum/mecanum_data.hpp"
#include <string>

class OdometryParameters {
public:
    explicit OdometryParameters(rclcpp::Node* node);

    DriveType getDriveType();

    MecanumParameters getMecanumParams();

private:
    rclcpp::Node* node_; 
};

#endif // ODOMETRY_PARAMETERS_HPP_