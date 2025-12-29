#include "ackermann/ackermann_kinematics.hpp"
#include "rclcpp/rclcpp.hpp" 
#include <cmath>
#include <cstdint> 

AckermannKinematics::AckermannKinematics() {
    state_.pose = {0.0, 0.0, 0.0};
    state_.velocity = {0.0, 0.0, 0.0};
    
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    first_run_ = true;
}

void AckermannKinematics::setConfig(const AckermannParameters& params) {
    params_ = params;
}

void AckermannKinematics::update(const AckermannStateData& inputs, double dt) {
    
    // RCLCPP_INFO(rclcpp::get_logger("AckermannKinematics"), "Delta time: %.6f", dt);
    // The Kaist Urban dataset prints the encoder_count topic at 100 Hz, which is equivalent to 10 ms.
    // The output of RCLCPP_INFO confirms this calculation.

    // Using %ld for 64-bit integer logging to match raw topic values perfectly
    RCLCPP_INFO(rclcpp::get_logger("AckermannKinematics"), 
                    "RAW TICKS -> Left: %ld, Right: %ld", 
                    inputs.left_ticks, 
                    inputs.right_ticks);

    if (dt < 0.0001) {
        return;
    }

    if (first_run_) {
        prev_left_ticks_ = inputs.left_ticks;
        prev_right_ticks_ = inputs.right_ticks;
        first_run_ = false;
        return;
    }

    // Integer subtraction to avoid precision errors with large tick values
    int64_t current_left_ticks = inputs.left_ticks;
    int64_t current_right_ticks = inputs.right_ticks;

    int64_t delta_left_ticks = current_left_ticks - prev_left_ticks_;
    int64_t delta_right_ticks = current_right_ticks - prev_right_ticks_;
    
    RCLCPP_INFO(rclcpp::get_logger("AckermannKinematics"), 
                "DELTA TICKS -> Left: %ld, Right: %ld", 
                delta_left_ticks, delta_right_ticks);
                
    prev_left_ticks_ = current_left_ticks;
    prev_right_ticks_ = current_right_ticks;

    float v_left = (static_cast<float>(delta_left_ticks) / (params_.ticks_per_rev * dt)) * (M_PI * params_.left_wheel_diameter);
    float v_right = (static_cast<float>(delta_right_ticks) / (params_.ticks_per_rev * dt)) * (M_PI * params_.right_wheel_diameter);

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