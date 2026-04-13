#ifndef DRIVER_NODE_HPP
#define DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

class DriverNode : public rclcpp::Node {
    public:
        DriverNode();
    private:
        void initialise_parameters();
        int max_speed_limit_; 
};

#endif