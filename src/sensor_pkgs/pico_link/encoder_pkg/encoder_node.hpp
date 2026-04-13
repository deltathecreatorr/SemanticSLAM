#ifndef ENCODER_NODE_HPP
#define ENCODER_NODE_HPP

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

class EncoderNode : public rclcpp::Node {
    public:
        EncoderNode();
    private:
        void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

        const double WHEEL_RADIUS = 0.0750;
        const double TRACK_WIDTH = 0.225;
        const double TICKS_PER_REV = 960.0;

        double x_;
        double y_; 
        double theta_;
        double prev_left_ticks_;
        double prev_right_ticks_;
        rclcpp::Time last_time_;
        bool first_read_;
};

#endif