#ifndef ACKERMANN_DATA_HPP_
#define ACKERMANN_DATA_HPP_

struct AckermannStateData {
    float left_ticks;  
    float right_ticks; 
};

struct AckermannParameters {    
    double left_wheel_diameter;
    double right_wheel_diameter;
    double wheelbase;     
    double ticks_per_rev; 
};

#endif // ACKERMANN_DATA_HPP_