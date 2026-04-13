#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class IMUNode : public rclcpp::Node {
    public:
        IMUNode();
    private:
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};