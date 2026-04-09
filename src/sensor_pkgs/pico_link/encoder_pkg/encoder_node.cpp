#include "encoder_node.hpp"
#include <cmath>

EncoderNode::EncoderNode() : Node("encoder_processor_node") {
    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel_ticks", 10, std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    RCLCPP_INFO(this->get_logger(), "Encoder Node Initialized");
}

void EncoderNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        RCLCPP_WARN(this->get_logger(), "Invalid encoder array size");
        return;
    }

    if (TICKS_PER_REV == 0) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
            "TICKS_PER_REV is 0! Please update the constants in encoder_node.hpp");
        return;
    }

    double left_ticks = (msg->data[0] + msg->data[1]) / 2.0;
    double right_ticks = (msg->data[2] + msg->data[3]) / 2.0;

    double distance_left = (left_ticks / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;
    double distance_right = (right_ticks / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;

    double distance_center = (distance_left + distance_right) / 2.0;
    
    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = distance_center; 
    
    odom_pub_->publish(odom);
}