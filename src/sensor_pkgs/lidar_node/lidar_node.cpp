#include <sensor_pkgs/lidar_node/lidar_node.hpp>
#include <cmath>

LidarNode::LidarNode() : Node("lidar_node") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",
        10,
        std::bind(&LidarNode::scan_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LidarNode On.");
}

void LidarNode::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    float min_distance = std::numeric_limits<float>::infinity();
    int closest_index = -1;

    // For all the range readings, find the closest one
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        float distance = msg->ranges[i];
        if (std::isfinite(distance) && distance > msg->range_min && distance < msg->range_max) {
            if (distance < min_distance) {
                min_distance = distance;
                closest_index = i;
            }
        }
    } 

    if (closest_index != -1) {
        float angle = msg->angle_min + closest_index * msg->angle_increment;
        this->min_distance_ = min_distance;
        this->current_x_ = min_distance * std::cos(angle);
        this->current_y_ = min_distance * std::sin(angle);
        this->data_received_ = true;
    }
}



