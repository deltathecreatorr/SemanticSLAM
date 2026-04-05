#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>


class LidarNode : public rclcpp::Node
{
public:
    LidarNode(); 
    float current_x_ = 0.0;
    float current_y_ = 0.0;
    float min_distance_ = 0.0;
    bool data_received_ = false;

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};