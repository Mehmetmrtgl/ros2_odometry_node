#ifndef VEHICLE_INTERFACE_HPP_
#define VEHICLE_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"

class VehicleInterface {
public:
    virtual ~VehicleInterface() = default;

    virtual void setup() = 0; 
};

#endif // VEHICLE_INTERFACE_HPP_