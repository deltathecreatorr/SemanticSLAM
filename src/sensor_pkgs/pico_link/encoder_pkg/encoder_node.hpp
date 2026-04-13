#ifndef ENCODER_NODE_HPP
#define ENCODER_NODE_HPP

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <mutex>

class EncoderNode : public rclcpp::Node {
    public:
        EncoderNode();
    private:
        std::mutex state_mutex_;

        rclcpp::CallbackGroup::SharedPtr encoder_cbg_;
        rclcpp::CallbackGroup::SharedPtr cmd_cbg_;

        void encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr sub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr motor_pub_;

        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

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

        double target_vel_l_ = 0.0;
        double target_vel_r_ = 0.0;

        const double Kp = 400.0;
        const double Ki = 100.0;
        const double Kd = 10.0;

        double e_Sum_l_r = 0.0, prev_e_l_ = 0.0;
        double e_Sum_r_r = 0.0, prev_e_r_ = 0.0;
};

#endif