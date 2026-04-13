#include "EncoderHandler/EncoderHandler.hpp"
#include "IMUHandler/IMUHandler.hpp"
#include "MotorDrivers/MotorDrivers.hpp"
#include <Arduino.h>

unsigned long lastStreamTime = 0;
const int PUBLISH_INTERVAL_MS = 100;

void setup() {
    Serial.begin(115200);

    EncoderHandler::initEncoders();
    EncoderHandler::resetEncoders();
    initMotors();

    while (!initIMU()) {
        Serial.println("Failed to initialize IMU. Retrying...");
        delay(1000);
    }
}

void loop() {

    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        if (cmd.startsWith("M,")) {
            int fl, rl, fr, rr;
            int parsed = sscanf(cmd.c_str(), "M,%d,%d,%d,%d", &fl, &rl, &fr, &rr);

            if (parsed == 4) {
                setMotorSpeeds(fl, rl, fr, rr);
            } else {
                Serial.println("Invalid motor command format. Expected: M,fl_speed,rl_speed,fr_speed,rr_speed");
            }
        }
    }

    if (millis() - lastStreamTime >= PUBLISH_INTERVAL_MS) {
        lastStreamTime = millis();
        streamIMUData();

        WheelTicks ticks = EncoderHandler::getEncoderTicks();
        Serial.print("E,");
        Serial.print(ticks.fl); Serial.print(",");
        Serial.print(ticks.rl); Serial.print(",");
        Serial.print(ticks.fr); Serial.print(",");
        Serial.println(ticks.rr);

    }
}

