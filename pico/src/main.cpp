#include <Arduino.h>
#include "MotorDrivers/MotorDrivers.hpp"
#include "EncoderHandler/EncoderHandler.hpp"
#include "IMUHandler/IMUHandler.hpp"

unsigned long lastUpdate = 0;
const int updateInterval = 20;

void setup() {
    Serial.begin(115200);
    initMotors();
    initEncoders();
    initIMU();

    Serial.println("READY");    
}

void loop() {
    unsigned long now = millis();

    if (now - lastUpdate >= updateInterval) {
        lastUpdate = now;
        streamIMUData();

        WheelTicks ticks = getEncoderTicks();
        Serial.print("ENCODER,");
        Serial.print(ticks.fl_ticks); Serial.print(",");
        Serial.print(ticks.rl_ticks); Serial.print(",");
        Serial.print(ticks.fr_ticks); Serial.print(",");
        Serial.println(ticks.rr_ticks);
    }

    if (Serial.available() > 0) {
        char header = Serial.read();

        if (header == 'M') {
            if (Serial.read() == ',') {
                int fl_speed = Serial.parseInt();
                int rl_speed = Serial.parseInt();
                int fr_speed = Serial.parseInt();
                int rr_speed = Serial.parseInt();
                setMotorSpeeds(fl_speed, rl_speed, fr_speed, rr_speed);
            }
        } else if (header == 'R') {
            resetEncoders();
        }
    }
}