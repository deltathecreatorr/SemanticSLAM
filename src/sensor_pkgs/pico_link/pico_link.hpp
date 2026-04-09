#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>

class PicoLink : public rclcpp::Node {
    public:
        PicoLink();
        ~PicoLink();
    private:
        void setup_serial();
        void read_loop();
        void process_line(const std::string& line);
        void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

        int fd_ = -1;
        std::string serial_buffer_;
        
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
        rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
        rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr enc_pub_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
};