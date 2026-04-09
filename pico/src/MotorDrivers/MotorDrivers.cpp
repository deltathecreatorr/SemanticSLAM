#include "MotorDrivers.hpp"

const int RR_IN1 = 2; const int RR_IN2 = 3;
const int FR_IN1 = 4; const int FR_IN2 = 5;
const int FL_IN1 = 6; const int FL_IN2 = 7;
const int RL_IN1 = 8; const int RL_IN2 = 9;

void initMotors() {
    int pins[] = {RR_IN1, RR_IN2, FR_IN1, FR_IN2, FL_IN1, FL_IN2, RL_IN1, RL_IN2};
    for(int p : pins) pinMode(p, OUTPUT);
    stopMotors();
}

void driveMotor(int in1, int in2, int speed) {
    int pwmValue = constrain(abs(speed), 0, 255);
    
    if (speed > 0) {
        analogWrite(in1, pwmValue);
        analogWrite(in2, 0);
    } else if (speed < 0) {
        analogWrite(in1, 0); 
        analogWrite(in2, pwmValue);
    } else {
        analogWrite(in1, 0);
        analogWrite(in2, 0);
    }
}

void setMotorSpeeds (int fl_speed, int rl_speed, int fr_speed, int rr_speed) {
    driveMotor(FL_IN1, FL_IN2, fl_speed);
    driveMotor(RL_IN1, RL_IN2, rl_speed);
    driveMotor(FR_IN1, FR_IN2, fr_speed);
    driveMotor(RR_IN1, RR_IN2, rr_speed);
}

void stopMotors() {
    analogWrite(FL_IN1, 0); analogWrite(FL_IN2, 0);
    analogWrite(RL_IN1, 0); analogWrite(RL_IN2, 0);
    analogWrite(FR_IN1, 0); analogWrite(FR_IN2, 0);
    analogWrite(RR_IN1, 0); analogWrite(RR_IN2, 0);
}