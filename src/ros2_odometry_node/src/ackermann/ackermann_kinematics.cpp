#include "ackermann/ackermann_kinematics.hpp"
#include <iostream> // Debug için

AckermannKinematics::AckermannKinematics() {
    state_.pose = {0.0, 0.0, 0.0};
    state_.velocity = {0.0, 0.0, 0.0};
    
    prev_left_ticks_ = 0.0;
    prev_right_ticks_ = 0.0;
    first_run_ = true;
}

void AckermannKinematics::setConfig(const AckermannParameters& params) {
    params_ = params;
}

void AckermannKinematics::update(const AckermannStateData& inputs, double dt) {
    
    if (dt < 0.0001) return;

    if (first_run_) {
        prev_left_ticks_ = inputs.left_ticks;
        prev_right_ticks_ = inputs.right_ticks;
        first_run_ = false;
        return;
    }

    double delta_left_ticks = inputs.left_ticks - prev_left_ticks_;
    double delta_right_ticks = inputs.right_ticks - prev_right_ticks_;

    prev_left_ticks_ = inputs.left_ticks;
    prev_right_ticks_ = inputs.right_ticks;


    float v_left = (delta_left_ticks / (params_.ticks_per_rev * dt)) * (M_PI * params_.left_wheel_diameter);
    
    float v_right = (delta_right_ticks / (params_.ticks_per_rev * dt)) * (M_PI * params_.right_wheel_diameter);

    float linear_v = (v_right + v_left) / 2.0f;

    float angular_v = 0.0f;
    if (params_.wheelbase > 0.001) {
        angular_v = (v_right - v_left) / params_.wheelbase;
    }

    float dx = linear_v * std::cos(state_.pose.theta) * dt;
    float dy = linear_v * std::sin(state_.pose.theta) * dt;
    float dtheta = angular_v * dt;

    state_.pose.x += dx;
    state_.pose.y += dy;
    state_.pose.theta += dtheta;

    state_.pose.theta = std::atan2(std::sin(state_.pose.theta), std::cos(state_.pose.theta));

    state_.velocity.vx = linear_v;
    state_.velocity.omega = angular_v;
}

RobotState AckermannKinematics::getState() const { 
    return state_; 
}