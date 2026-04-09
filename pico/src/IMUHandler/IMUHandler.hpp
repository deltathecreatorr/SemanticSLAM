#ifndef IMU_HANDLER_HPP
#define IMU_HANDLER_HPP

#include <Arduino.h>
#include "MPU9250.h"

bool initIMU();
void streamIMUData();

#endif