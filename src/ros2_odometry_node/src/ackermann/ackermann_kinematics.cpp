#include "ackermann/ackermann_kinematics.hpp"
#include "rclcpp/rclcpp.hpp" 
#include <cmath>
#include <cstdint> 

AckermannKinematics::AckermannKinematics() {
    // BAŞLANGIÇ DURUMU: (x, y, theta)
    // Robotu 180 derece (M_PI) döndürerek başlatıyoruz
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
    if (dt < 0.0001) return;

    if (first_run_) {
        prev_left_ticks_ = inputs.left_ticks;
        prev_right_ticks_ = inputs.right_ticks;
        first_run_ = false;
        return;
    }

    int64_t current_left_ticks = inputs.left_ticks;
    int64_t current_right_ticks = inputs.right_ticks;

    int64_t delta_left_ticks = current_left_ticks - prev_left_ticks_;
    int64_t delta_right_ticks = current_right_ticks - prev_right_ticks_;
                
    prev_left_ticks_ = current_left_ticks;
    prev_right_ticks_ = current_right_ticks;

    float v_left = (static_cast<float>(delta_left_ticks) / (params_.ticks_per_rev * dt)) * (M_PI * params_.left_wheel_diameter);
    float v_right = (static_cast<float>(delta_right_ticks) / (params_.ticks_per_rev * dt)) * (M_PI * params_.right_wheel_diameter);

    float linear_v = (v_right + v_left) / 2.0f;

    // Açısal hız (Sola dönüş pozitif olmalı)
    float angular_v = 0.0f;
    if (params_.wheelbase > 0.001) {
        // Eğer robot sola dönünce açı azalıyorsa v_left - v_right yapın
        angular_v = (v_right - v_left) / params_.wheelbase; 
    }

    float dtheta = angular_v * dt;
    float avg_theta = state_.pose.theta + (dtheta / 2.0f);

    float dx = linear_v * std::cos(avg_theta) * dt;
    float dy = linear_v * std::sin(avg_theta) * dt;

    state_.pose.x += dx;
    state_.pose.y += dy;
    state_.pose.theta += dtheta;

    // Normalizasyon [-PI, PI]
    state_.pose.theta = std::atan2(std::sin(state_.pose.theta), std::cos(state_.pose.theta));

    state_.velocity.vx = linear_v;
    state_.velocity.omega = angular_v;
}

RobotState AckermannKinematics::getState() const { 
    return state_; 
}