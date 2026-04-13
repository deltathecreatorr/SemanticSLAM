#include "encoder_node.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


EncoderNode::EncoderNode() : Node("encoder_processor_node") {
    sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
        "wheel_ticks", 10, std::bind(&EncoderNode::encoder_callback, this, std::placeholders::_1));

    cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&EncoderNode::cmd_callback, this, std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    motor_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("motor_commands", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    x_ = 0.0;
    y_ = 0.0;
    theta_ = 0.0;
    prev_left_ticks_ = 0.0;
    prev_right_ticks_ = 0.0;
    last_time_ = this->now();
    first_read_ = true;

    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->now();
    static_transform.header.frame_id = "base_link";   
    static_transform.child_frame_id = "base_laser";

    static_transform.transform.translation.x = 0.0;   
    static_transform.transform.translation.y = 0.0;  
    static_transform.transform.translation.z = 0.18;   
    static_transform.transform.rotation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Encoder Node Initialized");
}

void EncoderNode::cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    double linear_v = msg->linear.x;  
    double angular_v = msg->angular.z;

    target_vel_l_ = linear_v- (angular_v * TRACK_WIDTH / 2.0);
    target_vel_r_ = linear_v + (angular_v * TRACK_WIDTH / 2.0);

    if (linear_v == 0.0 && angular_v == 0.0) {
        e_Sum_l_r = 0.0;
        e_Sum_r_r = 0.0;
    }

    RCLCPP_DEBUG(this->get_logger(), "L: %.2f m/s, R: %.2f m/s", target_vel_l_, target_vel_r_);
}

void EncoderNode::encoder_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    if (msg->data.size() < 4) {
        return;
    }

    rclcpp::Time current_time = this->now();

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
    
    if (dt > 0) {
        if (target_vel_l_ == 0 && target_vel_r_ == 0) {
            e_Sum_l_r = 0.0;
            e_Sum_r_r = 0.0;
        }

        double v_l_actual = delta_left / dt;
        double v_r_actual = delta_right / dt;

        double error_l = target_vel_l_ - v_l_actual;
        e_Sum_l_r += error_l * dt;
        double d_error_l = (error_l - prev_e_l_) / dt;
        int pwm_l = (Kp * error_l) + (Ki * e_Sum_l_r) + (Kd * d_error_l);

        double error_r = target_vel_r_ - v_r_actual;
        e_Sum_r_r += error_r * dt;
        double d_error_r = (error_r - prev_e_r_) / dt;
        int pwm_r = (Kp * error_r) + (Ki * e_Sum_r_r) + (Kd * d_error_r);

        auto motor_msg = std_msgs::msg::Int32MultiArray();
        motor_msg.data = {
            std::max(-255, std::min(255, pwm_l)), 
            std::max(-255, std::min(255, pwm_r))
        };
        motor_pub_->publish(motor_msg);

        prev_e_l_ = error_l;
        prev_e_r_ = error_r;
        
    }
    
    x_ += delta_center * cos(theta_);
    y_ += delta_center * sin(theta_);
    theta_ += delta_theta;

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

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = current_time;
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";

    transform.transform.translation.x = x_;
    transform.transform.translation.y = y_;
    transform.transform.translation.z = 0.0;

    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(transform);

    prev_left_ticks_ = current_left_ticks;
    prev_right_ticks_ = current_right_ticks;
    last_time_ = current_time;

}