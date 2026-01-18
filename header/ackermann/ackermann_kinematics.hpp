#ifndef ACKERMANN_KINEMATICS_HPP_
#define ACKERMANN_KINEMATICS_HPP_

#include "ackermann/ackermann_data.hpp"
#include "ros2_odometry_node/robot_state.hpp"
#include <cmath>

class AckermannKinematics {
public:
    AckermannKinematics();
    void setConfig(const AckermannParameters& params);

    void update(const AckermannStateData& inputs, double dt);

    RobotState getState() const;

private:
    RobotState state_;
    AckermannParameters params_;

    double prev_left_ticks_;
    double prev_right_ticks_;
    bool first_run_; 
};

#endif