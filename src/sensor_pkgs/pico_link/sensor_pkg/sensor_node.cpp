#include "sensor_node.hpp"
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

    last_time_ = this->get_clock()->now();
    first_read_ = true;

    RCLCPP_INFO(this->get_logger(), "Encoder Node Initialized");
}

void EncoderNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    double linear_v = msg->linear.x;  
    double angular_v = msg->angular.z;

    std::lock_guard<std::mutex> lock(state_mutex_);

    target_vel_fl_ = target_vel_rl_ = linear_v - (angular_v * TRACK_WIDTH / 2.0);
    target_vel_fr_ = target_vel_rr_ = linear_v + (angular_v * TRACK_WIDTH / 2.0);

    if (std::abs(linear_v) < 0.001 && std::abs(angular_v) < 0.001) {
        e_sum_fl = e_sum_rl = e_sum_fr = e_sum_rr = 0.0;
    }

    // RCLCPP_INFO(this->get_logger(), "L: %.2f m/s, R: %.2f m/s", target_vel_l_, target_vel_r_);
}

void EncoderNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) return;

    rclcpp::Time current_time = this->get_clock()->now();
    std::lock_guard<std::mutex> lock(state_mutex_);

    double fl_t = msg->data[0]; double rl_t = msg->data[1];
    double fr_t = msg->data[2]; double rr_t = msg->data[3];

    if (first_read_) {
        prev_fl_ticks_ = fl_t; prev_rl_ticks_ = rl_t;
        prev_fr_ticks_ = fr_t; prev_rr_ticks_ = rr_t;
        last_time_ = current_time; first_read_ = false;
        return;
    }

    double dt = (current_time - last_time_).seconds();
    if (dt < 0.01) return;

    auto get_dist = [&](double curr, double prev) {
        return ((curr - prev) / TICKS_PER_REV) * 2.0 * M_PI * WHEEL_RADIUS;
    };

    auto run_pid = [&](double target, double dist, double &sum, double &prev_e) {
        if (std::abs(target) < 0.001) {
            sum = 0;
            return 0;
        }
        double actual_v = dist / dt;
        double error = target - actual_v;
        sum = std::max(-0.5, std::min(0.5, sum + (error * dt)));
        double d_error = (error - prev_e) / dt;
        int pwm = (Kp * error) + (Ki * sum) + (Kd * d_error);
        prev_e = error;

        int deadband = 30; 
        if ((target_vel_fl_ > 0 && target_vel_fr_ < 0) || (target_vel_fl_ < 0 && target_vel_fr_ > 0)) {
            deadband += 10;
        }

        // Only floor PWM when it's in the same direction as the target.
        // If the PID is braking (opposing sign), let it fall to zero naturally.
        // The old logic snapped braking corrections to -deadband, causing
        // rapid direction reversals, motor noise, and stuttering.
        if (target > 0 && pwm > 0 && pwm < deadband) pwm = deadband;
        else if (target < 0 && pwm < 0 && pwm > -deadband) pwm = -deadband;

        return std::max(-255, std::min(255, pwm));
    };

    int pwm_fl = run_pid(target_vel_fl_, get_dist(fl_t, prev_fl_ticks_), e_sum_fl, prev_e_fl_);
    int pwm_rl = run_pid(target_vel_rl_, get_dist(rl_t, prev_rl_ticks_), e_sum_rl, prev_e_rl_);
    int pwm_fr = run_pid(target_vel_fr_, get_dist(fr_t, prev_fr_ticks_), e_sum_fr, prev_e_fr_);
    int pwm_rr = run_pid(target_vel_rr_, get_dist(rr_t, prev_rr_ticks_), e_sum_rr, prev_e_rr_);

    auto motor_msg = std_msgs::msg::Int32MultiArray();
    motor_msg.data = {pwm_fl, pwm_rl, pwm_fr, pwm_rr};
    motor_pub_->publish(motor_msg);

    double d_l = (get_dist(fl_t, prev_fl_ticks_) + get_dist(rl_t, prev_rl_ticks_)) / 2.0;
    double d_r = (get_dist(fr_t, prev_fr_ticks_) + get_dist(rr_t, prev_rr_ticks_)) / 2.0;
    double d_c = (d_l + d_r) / 2.0;
    double d_th = (d_r - d_l) / TRACK_WIDTH;

    x_ += d_c * cos(theta_);
    y_ += d_c * sin(theta_);
    theta_ += d_th;

    while (theta_ > M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ < -M_PI) theta_ += 2.0 * M_PI;

    auto odom = nav_msgs::msg::Odometry();
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    tf2::Quaternion q; q.setRPY(0, 0, theta_);
    odom.pose.pose.orientation.x = q.x(); odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z(); odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = d_c / dt; odom.twist.twist.angular.z = d_th / dt;
    odom_pub_->publish(odom);

    prev_fl_ticks_ = fl_t; prev_rl_ticks_ = rl_t;
    prev_fr_ticks_ = fr_t; prev_rr_ticks_ = rr_t;
    last_time_ = current_time;
}