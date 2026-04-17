#ifndef IMU_NODE_HPP
#define IMU_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <string>

class IMUNode: public rclcpp::Node {
    public:
        IMUNode();
        ~IMUNode();
    private:
        void read_serial_and_publish();
        bool setup_serial(const std::string& port);

        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
        rclcpp::TimerBase::SharedPtr timer_;

        int serial_fd_;
        std::string serial_buffer_;
};

#endif