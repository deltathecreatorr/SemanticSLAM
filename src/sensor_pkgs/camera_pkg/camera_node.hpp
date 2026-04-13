#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraNode : public rclcpp::Node {
    public:
        CameraNode();

        bool frame_received_ = false;
        int image_width_ = 0;
        int image_height_ = 0;
        std::string encoding_ = "";

    private:
        void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};