#include "encoder_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


EncoderNode::EncoderNode() : Node("encoder_processor_node") {

    encoder_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    cmd_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto encoder_opt = rclcpp::SubscriptionOptions();
    encoder_opt.callback_group = encoder_cbg_;
    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel_ticks", 10, std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1), encoder_opt);

    auto cmd_opt = rclcpp::SubscriptionOptions();
    cmd_opt.callback_group = cmd_cbg_;
    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&EncoderNode::cmd_callback, this, std::placeholders::_1), cmd_opt);
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom/unfiltered", 10);

    motor_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_commands", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    prev_left_ticks_ = 0.0;
    prev_right_ticks_ = 0.0;

    last_time_ = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    first_read_ = true;

    RCLCPP_INFO(this->get_logger(), "Encoder Node Initialized");
}

void EncoderNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    double linear_v = msg->linear.x;  
    double angular_v = msg->angular.z;

    std::lock_guard<std::mutex> lock(state_mutex_);

    target_vel_l_ = linear_v- (angular_v * TRACK_WIDTH / 2.0);
    target_vel_r_ = linear_v + (angular_v * TRACK_WIDTH / 2.0);

    if (std::abs(linear_v) < 0.001 && std::abs(angular_v) < 0.001) {
        e_Sum_l_r = 0.0;
        e_Sum_r_r = 0.0;
    }

    // RCLCPP_INFO(this->get_logger(), "L: %.2f m/s, R: %.2f m/s", target_vel_l_, target_vel_r_);
}

void EncoderNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        return;
    }

    rclcpp::Time current_time = this->get_clock()->now();

    std::lock_guard<std::mutex> lock(state_mutex_);

    double current_left_ticks = (msg->data[0] + msg->data[1]) / 2.0;
    double current_right_ticks = (msg->data[2] + msg->data[3]) / 2.0;

    if (first_read_) {
        prev_left_ticks_ = current_left_ticks;
        prev_right_ticks_ = current_right_ticks;
        last_time_ = current_time;
        first_read_ = false;
        return;
    }

    double delta_left_ticks = current_left_ticks - prev_left_ticks_;
    double delta_right_ticks = current_right_ticks - prev_right_ticks_;

    double delta_left = (delta_left_ticks / TICKS_PER_REV) * 2 * M_PI * WHEEL_RADIUS;
    double delta_right = (delta_right_ticks / TICKS_PER_REV) * 2 * M_PI * WHEEL_RADIUS;

    double delta_center = (delta_left + delta_right) / 2.0;
    double delta_theta = (delta_right - delta_left) / TRACK_WIDTH;

    double dt = (current_time - last_time_).seconds();
    
    if (dt >= 0.01) {
        double v_l_actual = delta_left / dt;
        double v_r_actual = delta_right / dt;

        int pwm_l = 0;
        int pwm_r = 0;

        double error_l = target_vel_l_ - v_l_actual;
        e_Sum_l_r += error_l * dt;
        double max_i = 50.0; 
        e_Sum_l_r = std::max(-max_i, std::min(max_i, e_Sum_l_r));
        double d_error_l = (error_l - prev_e_l_) / dt;
        pwm_l = (Kp * error_l) + (Ki * e_Sum_l_r) + (Kd * d_error_l);
        prev_e_l_ = error_l;

        double error_r = target_vel_r_ - v_r_actual;
        e_Sum_r_r += error_r * dt;
        e_Sum_r_r = std::max(-max_i, std::min(max_i, e_Sum_r_r));
        double d_error_r = (error_r - prev_e_r_) / dt;
        pwm_r = (Kp * error_r) + (Ki * e_Sum_r_r) + (Kd * d_error_r);
        prev_e_r_ = error_r;

       
        int min_pwm = 120;
        int rotation_boost = 30;

        if (std::abs(target_vel_l_) < 0.001) {
            pwm_l = 0;
            e_Sum_l_r = 0.0; 
        } else {
            if (std::abs(target_vel_l_) > 0 && std::abs(target_vel_r_) > 0 && 
            ((target_vel_l_ > 0 && target_vel_r_ < 0) || (target_vel_l_ < 0 && target_vel_r_ > 0))) {
                min_pwm += rotation_boost;
            }
            if (target_vel_l_ > 0.0 && pwm_l < min_pwm) pwm_l = min_pwm;
            else if (target_vel_l_ < 0.0 && pwm_l > -min_pwm) pwm_l = -min_pwm;
        }

        if (std::abs(target_vel_r_) < 0.001) {
            pwm_r = 0;
            e_Sum_r_r = 0.0;
        } else {
            if (target_vel_r_ > 0.0 && pwm_r < min_pwm) pwm_r = min_pwm;
            else if (target_vel_r_ < 0.0 && pwm_r > -min_pwm) pwm_r = -min_pwm;
        }

        auto motor_msg = std_msgs::msg::Int32MultiArray();
        motor_msg.data = {
            std::max(-255, std::min(255, pwm_l)), 
            std::max(-255, std::min(255, pwm_r))
        };

        if ((std::abs(pwm_l) > 200 && delta_left_ticks == 0) || (std::abs(pwm_r) > 200 && delta_right_ticks == 0)) {
            RCLCPP_WARN(this->get_logger(), "STALL DETECTED - FORCING MOTOR STOP");
            pwm_l = 0;
            pwm_r = 0;
            e_Sum_l_r = 0.0;
            e_Sum_r_r = 0.0;
            motor_msg.data = {0, 0};
        }
        motor_pub_->publish(motor_msg);
        
    }
    
    x_ += delta_center * cos(theta_);
    y_ += delta_center * sin(theta_);
    theta_ += delta_theta;

    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();

    if (dt > 0) {
        odom.twist.twist.linear.x = delta_center / dt;
        odom.twist.twist.angular.z = delta_theta / dt;
    }

    odom_pub_->publish(odom);

    std::vector<geometry_msgs::msg::TransformStamped> transforms;

    tf_broadcaster_->sendTransform(transforms);

    prev_left_ticks_ = current_left_ticks;
    prev_right_ticks_ = current_right_ticks;
    last_time_ = current_time;

}