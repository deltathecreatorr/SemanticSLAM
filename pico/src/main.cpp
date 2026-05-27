#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>
#include <rmw_microros/rmw_microros.h>
#include <MadgwickAHRS.h>

#include "EncoderHandler/EncoderHandler.hpp"
#include "IMUHandler/IMUHandler.hpp"
#include "MotorDrivers/MotorDrivers.hpp"
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu imu_msg;

rcl_publisher_t enc_publisher;      
std_msgs__msg__Int32MultiArray enc_msg;

rcl_subscription_t motor_subscriber;
std_msgs__msg__Int32MultiArray motor_cmd_msg;
rclc_executor_t executor;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

Madgwick filter;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){};}}

unsigned long lastStreamTime = 0;
const int PUBLISH_INTERVAL_MS = 50;

void motor_callback(const void * msgin) {
    const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;

    if (msg->data.size >= 4) {
        setMotorSpeeds(msg->data.data[0], msg->data.data[1], msg->data.data[2], msg->data.data[3]);
    }
}

void setup() {
    Serial.begin(115200);
    set_microros_serial_transports(Serial);
    
    EncoderHandler::initEncoders();
    EncoderHandler::resetEncoders();
    initMotors();
    while (!initIMU()) { delay(1000); }

    allocator = rcl_get_default_allocator();
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        delay(100);
    }

    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

     while (!rmw_uros_epoch_synchronized()) {
        rmw_uros_sync_session(1000);
        delay(100);
    }

    RCCHECK(rclc_node_init_default(&node, "pico_node", "", &support));

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data"));

    imu_msg.header.frame_id.data = (char*)"imu_link";
    imu_msg.header.frame_id.size = strlen(imu_msg.header.frame_id.data);

    filter.begin(100.0);

    RCCHECK(rclc_publisher_init_default(&enc_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "wheel_ticks"));

    enc_msg.data.capacity = 4;
    enc_msg.data.size = 4;
    enc_msg.data.data = (int32_t*) malloc(enc_msg.data.capacity * sizeof(int32_t));

    enc_msg.layout.dim.capacity = 0;
    enc_msg.layout.dim.size = 0;
    enc_msg.layout.dim.data = NULL;

    RCCHECK(rclc_subscription_init_default(
        &motor_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "motor_commands"));

    motor_cmd_msg.data.capacity = 4;
    motor_cmd_msg.data.size = 0;
    motor_cmd_msg.data.data = (int32_t*) malloc(motor_cmd_msg.data.capacity * sizeof(int32_t));
    motor_cmd_msg.layout.dim.capacity = 0;
    motor_cmd_msg.layout.dim.size = 0;
    motor_cmd_msg.layout.dim.data = NULL;

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(
        &executor, &motor_subscriber, &motor_cmd_msg, &motor_callback, ON_NEW_DATA));

}

void loop() {
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    unsigned long current_time = millis();

    static unsigned long last_ping_time = 0;
    if (millis() - last_ping_time > 2000) {
        last_ping_time = millis();
        
        if (rmw_uros_ping_agent(100, 3) != RMW_RET_OK) {
            stopMotors();
            delay(500);
            NVIC_SystemReset(); 
        }
    }

    static unsigned long last_filter_time = 0;
    if (current_time - last_filter_time >= 10) {
        last_filter_time = current_time;

        if (populateIMUMsg(&imu_msg)) {
            float gx = imu_msg.angular_velocity.x;
            float gy = imu_msg.angular_velocity.y;
            float gz = imu_msg.angular_velocity.z;
            float ax = imu_msg.linear_acceleration.x;
            float ay = imu_msg.linear_acceleration.y;
            float az = imu_msg.linear_acceleration.z;

            float gx_deg = gx * 180.0 / M_PI;
            float gy_deg = gy * 180.0 / M_PI;
            float gz_deg = gz * 180.0 / M_PI;

            filter.updateIMU(gx_deg, gy_deg, gz_deg, ax, ay, az);
        }
    }

    if (current_time - lastStreamTime >= PUBLISH_INTERVAL_MS) {
        lastStreamTime = current_time;

        int64_t time_ns = rmw_uros_epoch_nanos();
        imu_msg.header.stamp.sec = time_ns / 1000000000;
        imu_msg.header.stamp.nanosec = time_ns % 1000000000;

        float roll = filter.getRollRadians();
        float pitch = filter.getPitchRadians();
        float yaw = filter.getYawRadians();

        float cr = cos(roll *0.5f);
        float sr = sin(roll *0.5f);
        float cp = cos(pitch *0.5f);
        float sp = sin(pitch *0.5f);
        float cy = cos(yaw *0.5f);
        float sy = sin(yaw *0.5f);

        imu_msg.orientation.w = cr * cp * cy + sr * sp * sy;
        imu_msg.orientation.x = sr * cp * cy - cr * sp * sy;
        imu_msg.orientation.y = cr * sp * cy + sr * cp * sy;
        imu_msg.orientation.z = cr * cp * sy - sr * sp * cy;

        RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));

        WheelTicks ticks = EncoderHandler::getEncoderTicks();
        enc_msg.data.data[0] = ticks.fl;
        enc_msg.data.data[1] = ticks.rl;
        enc_msg.data.data[2] = ticks.fr;
        enc_msg.data.data[3] = ticks.rr;
        
        RCSOFTCHECK(rcl_publish(&enc_publisher, &enc_msg, NULL));
    }
}