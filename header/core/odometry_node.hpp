#ifndef ODOMETRY_NODE_HPP_
#define ODOMETRY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "ros2_odometry_node/vehicle_interface.hpp"
#include <memory>

class OdometryNode : public rclcpp::Node {
public:
    OdometryNode();
    ~OdometryNode() = default;

private:
    std::unique_ptr<VehicleInterface> vehicle_impl_;
};

#endif