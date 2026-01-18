#include "ackermann/ackermann_kinematics.hpp"
#include "rclcpp/rclcpp.hpp" 
#include <cmath>
#include <cstdint> 

AckermannKinematics::AckermannKinematics() {
    state_.pose = {0.0, 0.0, M_PI}; 
    state_.velocity = {0.0, 0.0, 0.0};
    
    prev_left_ticks_ = 0;
    prev_right_ticks_ = 0;
    first_run_ = true;
}

void AckermannKinematics::setConfig(const AckermannParameters& params) {
    params_ = params;
}

void AckermannKinematics::update(const AckermannStateData& inputs, double dt) {
    if (dt < 1e-6) return;
    
    if (first_run_) {
        prev_left_ticks_ = inputs.left_ticks;
        prev_right_ticks_ = inputs.right_ticks;
        first_run_ = false;
        return;
    }
    
    // Calculate tick differences
    int64_t delta_left_ticks = inputs.left_ticks - prev_left_ticks_;
    int64_t delta_right_ticks = inputs.right_ticks - prev_right_ticks_;
    
    prev_left_ticks_ = inputs.left_ticks;
    prev_right_ticks_ = inputs.right_ticks;
    
    // Calculate wheel distances (use double for better precision)
    double left_revs = static_cast<double>(delta_left_ticks) / params_.ticks_per_rev;
    double right_revs = static_cast<double>(delta_right_ticks) / params_.ticks_per_rev;
    
    double left_distance = left_revs * M_PI * params_.left_wheel_diameter;
    double right_distance = right_revs * M_PI * params_.right_wheel_diameter;
    
    // Calculate velocities
    double v_left = left_distance / dt;
    double v_right = right_distance / dt;
    
    double linear_v = (v_left + v_right) / 2.0;
    double angular_v = (v_right - v_left) / params_.wheelbase;
    
    // Store velocities
    state_.velocity.vx = linear_v;
    state_.velocity.omega = angular_v;
    
    // Integrate position using exact arc equations
    double dx, dy, dtheta;
    
    if (std::abs(angular_v) < 1e-6) {
        // Straight line motion
        dtheta = 0.0;
        dx = linear_v * std::cos(state_.pose.theta) * dt;
        dy = linear_v * std::sin(state_.pose.theta) * dt;
    } else {
        // Arc motion - exact solution
        dtheta = angular_v * dt;
        double radius = linear_v / angular_v;
        
        dx = radius * (std::sin(state_.pose.theta + dtheta) - std::sin(state_.pose.theta));
        dy = radius * (-std::cos(state_.pose.theta + dtheta) + std::cos(state_.pose.theta));
    }
    
    // Update pose
    state_.pose.x += dx;
    state_.pose.y += dy;
    state_.pose.theta += dtheta;
    
    // Normalize angle to [-π, π]
    state_.pose.theta = std::atan2(std::sin(state_.pose.theta), std::cos(state_.pose.theta));
}

RobotState AckermannKinematics::getState() const { 
    return state_; 
}