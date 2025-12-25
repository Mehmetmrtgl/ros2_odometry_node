#ifndef MECANUM_KINEMATICS_HPP_
#define MECANUM_KINEMATICS_HPP_

#include "ros2_odometry_node/robot_state.hpp"   
#include "mecanum/mecanum_data.hpp"  
#include <cmath>

class MecanumKinematics {
public:
    MecanumKinematics();
    
    void setConfig(const MecanumParameters& params);
    void update(const MecanumWheelStates& inputs);
    RobotState getState() const;

private:
    RobotState state_; 
    MecanumParameters params_;

    float rpm_to_rads(float rpm);
};

#endif // MECANUM_KINEMATICS_HPP_