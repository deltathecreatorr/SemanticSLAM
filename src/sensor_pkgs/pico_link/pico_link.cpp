#include "pico_link.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <fmt/core.h>
#include <algorithm>

PicoLink::PicoLink() : Node("pico_link") {
    setup_serial();

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
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

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
    if (line.rfind("IMU,",0) == 0) {
        float ax, ay, az, gx, gy, gz, mx, my, mz;
        
        int parsed = sscanf(line.c_str(), "IMU,%f,%f,%f,%f,%f,%f,%f,%f,%f", &ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        
        if (parsed == 9) {
            auto msg = sensor_msgs::msg::Imu();
            msg.header.stamp = this->now();
            msg.header.frame_id = "imu_link";
            msg.linear_acceleration.x = ax;
            msg.linear_acceleration.y = ay;
            msg.linear_acceleration.z = az;
            msg.angular_velocity.x = gx;
            msg.angular_velocity.y = gy;
            msg.angular_velocity.z = gz;

            imu_pub_->publish(msg);

            auto mag_msg = sensor_msgs::msg::MagneticField();
            mag_msg.header.stamp = this->now();
            mag_msg.header.frame_id = "imu_link";
            mag_msg.magnetic_field.x = mx;
            mag_msg.magnetic_field.y = my;
            mag_msg.magnetic_field.z = mz;

            mag_pub_->publish(mag_msg);
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "SUCCESS: Publishing /imu/data_raw!");
        } else {
            RCLCPP_WARN(this->get_logger(), "Parse Failed! sscanf only found %d variables.", parsed);
        }
    } 
    else if (line.rfind("E,",0) == 0) {
        long fl, rl, fr, rr;
        int parsed = sscanf(line.c_str(), "E,%ld,%ld,%ld,%ld", &fl, &rl, &fr, &rr);
        
        if (parsed == 4) {
            auto msg = std_msgs::msg::Int32MultiArray();
            msg.data = {static_cast<int>(fl), static_cast<int>(rl), static_cast<int>(fr), static_cast<int>(rr)};
            enc_pub_->publish(msg);
        }
    }
}

void PicoLink::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (fd_ == -1) return;

    int max_speed = this->get_parameter("MAX_SPEED").as_int();

    int left = static_cast<int>((msg->linear.x - msg->angular.z) * max_speed);
    int right = static_cast<int>((msg->linear.x + msg->angular.z) * max_speed);

    left = std::clamp(left, -255, 255);
    right = std::clamp(right, -255, 255);

    std::string cmd = fmt::format("M,{},{},{},{}\n", left, left, right, right);
    write(fd_, cmd.c_str(), cmd.size());
}