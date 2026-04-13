#include "IMU_node.hpp"

IMUNode::IMUNode() : Node("imu_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10, std::bind(&IMUNode::imu_callback, this, std::placeholders::_1));
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);
}

void IMUNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    auto processed_msg = *msg;
    processed_msg.header.frame_id = "imu_link";
    pub_->publish(processed_msg);
}