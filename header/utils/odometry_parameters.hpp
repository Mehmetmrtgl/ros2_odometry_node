#ifndef ODOMETRY_PARAMETERS_HPP_
#define ODOMETRY_PARAMETERS_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_odometry_node/enums.hpp"
#include "mecanum/mecanum_data.hpp"
#include "ackermann/ackermann_data.hpp"

class OdometryParameters {
public:
    explicit OdometryParameters(rclcpp::Node* node);

    DriveType getDriveType(); 
    
    MecanumParameters getMecanumParams();
    AckermannParameters getAckermannParams();

private:
    rclcpp::Node* node_;
    std::string current_drive_type_; 

    void declareMecanum();
    void declareAckermann();
};

#endif // ODOMETRY_PARAMETERS_HPP_