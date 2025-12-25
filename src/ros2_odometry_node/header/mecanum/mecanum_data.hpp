#ifndef MECANUM_DATA_HPP_
#define MECANUM_DATA_HPP_

#include <array>

struct MecanumWheelStates {
    std::array<float, 4> ticks; 
    std::array<float, 4> rpm;   
};

struct MecanumParameters {
    double ticks_per_rev;
    double wheel_radius;
    float robot_l;
    float robot_w;
    float angle_epsilon;
};

#endif // MECANUM_DATA_HPP_