#include <sensor_pkgs/camera_pkg/camera_node.hpp>

CameraNode::CameraNode() : Node("camera_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&CameraNode::image_callback, this, std::placeholders::_1)
    );
}

void CameraNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->image_width_ = msg->width;
        this->image_height_ = msg->height;
        this->encoding_ = msg->encoding;
        this->frame_received_ = true;
}
