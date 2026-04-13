#include "encoder_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>

EncoderNode::EncoderNode() : Node("encoder_processor_node") {
    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel_ticks", 10, std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    prev_left_ticks_ = 0.0;
    prev_right_ticks_ = 0.0;
    last_time_ = this->now();
    first_read_ = true;

    RCLCPP_INFO(this->get_logger(), "Encoder Node Initialized");
}

void EncoderNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        return;
    }

    rclcpp::Time current_time = this->now();

    double current_left_ticks = (msg->data[0] + msg->data[1]) / 2.0;
    double current_right_ticks = (msg->data[2] + msg->data[3]) / 2.0;

    if (first_read_) {
        prev_left_ticks_ = current_left_ticks;
        prev_right_ticks_ = current_right_ticks;
        last_time_ = current_time;
        first_read_ = false;
        return;
    }

    double delta_left_ticks = current_left_ticks - prev_left_ticks_;
    double delta_right_ticks = current_right_ticks - prev_right_ticks_;

    double delta_left = (delta_left_ticks / TICKS_PER_REV) * 2 * M_PI * WHEEL_RADIUS;
    double delta_right = (delta_right_ticks / TICKS_PER_REV) * 2 * M_PI * WHEEL_RADIUS;

    double delta_center = (delta_left + delta_right) / 2.0;
    double delta_theta = (delta_right - delta_left) / TRACK_WIDTH;

    x_ += delta_center * cos(theta_);
    y_ += delta_center * sin(theta_);

    theta_ += delta_theta;

    double dt = (current_time - last_time_).seconds();
    double v_x = 0.0;
    double v_theta = 0.0;
    if (dt > 0) {
        v_x = delta_center / dt;
        v_theta = delta_theta / dt;
    }
    last_time_ = current_time;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.angular.z = v_theta;

    odom_pub_->publish(odom);




}