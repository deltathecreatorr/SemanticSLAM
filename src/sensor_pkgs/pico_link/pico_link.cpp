#include "pico_link.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fmt/core.h>
#include <algorithm>

PicoLink::PicoLink() : Node("pico_link") {
    setup_serial();

    this->declare_parameter("MAX_SPEED", 120);

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 10);
    enc_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("wheel_ticks", 10);

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&PicoLink::cmd_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&PicoLink::read_loop, this)
    );
} 

PicoLink::~PicoLink() {
    if (fd_ != -1) {
        close(fd_);
    }
}


void PicoLink::setup_serial() {
    fd_ = open("/dev/ttyACM0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port for cleanup");
        return;
    }
    struct termios tty;
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, B500000);
    cfsetospeed(&tty, B500000);

    tty.c_cflag |= (CLOCAL | CREAD | CS8);
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 
    tty.c_oflag &= ~OPOST;
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);

    tcsetattr(fd_, TCSANOW, &tty);
    fcntl(fd_, F_SETFL, FNDELAY);
}

void PicoLink::read_loop() {
    if (fd_ == -1) return;

    char buf[256];
    int n = read(fd_, buf, sizeof(buf) - 1);
    
    if (n > 0) {
        buf[n] = '\0';
        serial_buffer_ += buf; 

        size_t pos;
        while ((pos = serial_buffer_.find('\n')) != std::string::npos) {
            std::string line = serial_buffer_.substr(0, pos);
            serial_buffer_.erase(0, pos + 1);
            
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            if (!line.empty()) {
                process_line(line);
            }
        }
    }
}

void PicoLink::process_line(const std::string& line) {
    std::stringstream ss(line);
    std::string token;
    std::vector<std::string> values;

    while (std::getline(ss, token , ',')) {
        values.push_back(token);
    }

    if (values.empty()) {
        return;
    }

    if (values[0] == "IMU") {
        if (values.size() >= 7) {
            try {
                auto msg = sensor_msgs::msg::Imu();
                msg.header.stamp = this->now();
                msg.header.frame_id = "imu_link";

                auto safe_f = [](const std::string& s) {
                    try {
                        return s.empty() || s == "-" ? 0.0f : std::stof(s); 
                    } catch (...) {
                        return 0.0f;
                    }
                };

                msg.linear_acceleration.x = safe_f(values[1]);
                msg.linear_acceleration.y = safe_f(values[2]);
                msg.linear_acceleration.z = safe_f(values[3]);

                msg.angular_velocity.x = safe_f(values[4]);
                msg.angular_velocity.y = safe_f(values[5]);
                
                msg.angular_velocity.z = safe_f(values[6]) * -1.0f;

                imu_pub_->publish(msg);

            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing IMU data: %s", e.what());
            }
        }
    }

    else if (values[0] == "E") {
        if (values.size() >= 5) {
            try {
                auto msg = std_msgs::msg::Int32MultiArray();
                msg.data = {
                    std::stoi(values[1]),
                    std::stoi(values[2]),
                    std::stoi(values[3]),
                    std::stoi(values[4])
                };
                enc_pub_->publish(msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Error parsing encoder data: %s", e.what());
            }
        }
    }

}

void PicoLink::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (fd_ == -1) return;

    int max_speed = this->get_parameter("MAX_SPEED").as_int();

    const int MIN_PWM = 60;

    int left = static_cast<int>((msg->linear.x - msg->angular.z) * max_speed);
    int right = static_cast<int>((msg->linear.x + msg->angular.z) * max_speed);

    auto apply_min_pwm = [MIN_PWM](int speed) {
        if (speed == 0) return 0; 
        if (speed > 0 && speed < MIN_PWM) return MIN_PWM;
        if (speed < 0 && speed > -MIN_PWM) return -MIN_PWM;
        return speed;
    };

    left = apply_min_pwm(left);
    right = apply_min_pwm(right);

    left = std::clamp(left, -255, 255);
    right = std::clamp(right, -255, 255);

    std::string cmd = fmt::format("M,{},{},{},{}\n", left, left, right, right);
    write(fd_, cmd.c_str(), cmd.size());
}