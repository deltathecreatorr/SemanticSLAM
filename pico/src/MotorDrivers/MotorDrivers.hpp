#ifndef MOTOR_DRIVERS_HPP
#define MOTOR_DRIVERS_HPP

#include <Arduino.h>

void initMotors();
void setMotorSpeeds(int fl_speed, int rl_speed, int fr_speed, int rr_speed);
void stopMotors();

#endif
