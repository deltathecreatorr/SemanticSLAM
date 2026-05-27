#ifndef IMU_HANDLER_HPP
#define IMU_HANDLER_HPP

#include <Arduino.h>
#include "MPU9250.h"

#include <sensor_msgs/msg/imu.h>

bool initIMU();
bool populateIMUMsg(sensor_msgs__msg__Imu* msg);

#endif