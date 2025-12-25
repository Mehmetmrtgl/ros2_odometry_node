#include "ros2_odometry_node/mecanum_kinematics.hpp"
#include <cmath>

MecanumKinematics::MecanumKinematics() {
    state_.pose = {0.0f, 0.0f, 0.0f};
    state_.velocity = {0.0f, 0.0f, 0.0f};

    params_ = {4200.0, 0.04, 0.20, 0.23, 1e-4};
}

void MecanumKinematics::setConfig(const MecanumParameters& params) {
    params_ = params;
}

float MecanumKinematics::rpm_to_rads(float rpm) {
    return (rpm * 2 * M_PI) / 60.0;
}

void MecanumKinematics::update(const MecanumWheelStates& inputs) {
    
    float d[4];
    for(int i=0; i<4; i++) {
        d[i] = (2 * M_PI * params_.wheel_radius * inputs.ticks[i]) / params_.ticks_per_rev;
    }

    float dx_robot = (d[0] + d[1] + d[2] + d[3]) / 4.0;
    float dy_robot = (d[0] - d[1] - d[2] + d[3]) / 4.0;
    float dtheta   = (-d[0] + d[1] - d[2] + d[3]) / (4 * (params_.robot_l + params_.robot_w));

    state_.pose.x += dx_robot * cos(state_.pose.theta) - dy_robot * sin(state_.pose.theta);
    state_.pose.y += dy_robot * cos(state_.pose.theta) + dx_robot * sin(state_.pose.theta);

    if (std::fabs(dtheta) > params_.angle_epsilon) {
        state_.pose.theta += dtheta;
        state_.pose.theta = std::atan2(std::sin(state_.pose.theta), std::cos(state_.pose.theta));
    }

    float w[4];
    for(int i=0; i<4; i++) {
        w[i] = rpm_to_rads(inputs.rpm[i]);
    }

    state_.velocity.vx = (params_.wheel_radius / 4.0) * (w[0] + w[1] + w[2] + w[3]);
    state_.velocity.vy = (params_.wheel_radius / 4.0) * (w[0] - w[1] - w[2] + w[3]);
    state_.velocity.omega = (params_.wheel_radius / (4.0 * (params_.robot_l + params_.robot_w))) * (-w[0] + w[1] - w[2] + w[3]);
}

RobotState MecanumKinematics::getState() const {
    return state_;
}