#include "driver_node.hpp"

DriverNode::DriverNode() : Node("robot_manager_node") {
    this->initialise_parameters();

    RCLCPP_INFO(this->get_logger(), "Robot Manager Node Initialized");

}

void DriverNode::initialise_parameters() {
    this->declare_parameter("MAX_SPEED", 120);
    max_speed_limit_ = this->get_parameter("MAX_SPEED").as_int();
}