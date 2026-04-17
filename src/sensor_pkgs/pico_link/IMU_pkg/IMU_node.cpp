#include "IMU_node.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sstream>
#include <vector>

IMUNode::IMUNode() : Node("imu_node"), serial_fd_(-1) {
    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);

    if (setup_serial("/dev/ttyACM0")) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2),
            std::bind(&IMUNode::read_serial_and_publish, this)
        );
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    }

}

IMUNode::~IMUNode() {
    if (serial_fd_ >= 0) {
        close(serial_fd_);
    }
}

bool IMUNode::setup_serial(const std::string& port) {
    serial_fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

    if (serial_fd_ < 0) {
        return false;
    }

    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
        return false;
    }

    cfsetospeed(&tty, B500000);
    cfsetispeed(&tty, B500000);

    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO;
    tty.c_lflag &= ~ISIG;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_oflag &= ~OPOST;

    tcsetattr(serial_fd_, TCSANOW, &tty);
    return true;
}

void IMUNode::read_serial_and_publish() {
    if (serial_fd_ < 0) {
        return;
    }
    char read_buf[1024];
    int num_bytes = read(serial_fd_, &read_buf, sizeof(read_buf));

    if (num_bytes > 0) {
        serial_buffer_.append(read_buf, num_bytes);

        if (serial_buffer_.size() > 2048) serial_buffer_.clear();

        size_t pos;
        while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
            std::string line = serial_buffer_.substr(0, pos);
            serial_buffer_.erase(0, pos + 1);

            line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());

            if (line.rfind("IMU,", 0) == 0) {
                std::stringstream ss(line);
                std::string token;
                std::vector<std::string> values;

                while (std::getline(ss, token, ',')) {
                    values.push_back(token);
                }

                if (values.size() >= 7) {
                    try {
                        auto msg = sensor_msgs::msg::Imu();

                        auto safe_stod = [](const std::string& str) {
                            if (str.empty() || str == "-" || str == "." || str.find_first_not_of("0123456789.-") != std::string::npos) {
                                return 0.0;
                            }
                            return std::stod(str);
                        };

                        msg.header.frame_id = "imu_link";
                        msg.header.stamp = this->get_clock()->now();
                        msg.linear_acceleration.x = safe_stod(values[1]);
                        msg.linear_acceleration.y = safe_stod(values[2]);
                        msg.linear_acceleration.z = safe_stod(values[3]);

                        msg.angular_velocity.x = safe_stod(values[4]);
                        msg.angular_velocity.y = safe_stod(values[5]);
                    
                        msg.angular_velocity.z = safe_stod(values[6]) * -1.0;

                        pub_->publish(msg);
                    } catch (const std::exception& e) {
                        RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU data: %s", line.c_str());
                    }
                }
            }
        }
    }
}