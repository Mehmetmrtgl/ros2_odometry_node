#ifndef ROBOT_STATE_HPP_
#define ROBOT_STATE_HPP_

struct RobotPose {
    float x;
    float y;
    float theta; 
};

struct RobotVel {
    float vx;
    float vy;
    float omega;
};


struct RobotState {
    RobotPose pose;
    RobotVel velocity;
};

#endif // ROBOT_STATE_HPP_