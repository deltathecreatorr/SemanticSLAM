#include "EncoderHandler/EncoderHandler.hpp"
#include "IMUHandler/IMUHandler.hpp"
#include "MotorDrivers/MotorDrivers.hpp"
#include <Arduino.h>

unsigned long lastStreamTime = 0;
const int PUBLISH_INTERVAL_MS = 20;

char inputBuffer[64];
int bufferIndex = 0;

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

void processCommand(char* cmd) {
    if (cmd[0] == 'M') {
        int fl, rl, fr, rr;
        int parsed = sscanf(cmd, "M,%d,%d,%d,%d", &fl, &rl, &fr, &rr);
        if (parsed == 4) {
            setMotorSpeeds(fl, rl, fr, rr);
        }
    }
}

void loop() {

    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            inputBuffer[bufferIndex] = '\0'; // Terminate string
            processCommand(inputBuffer);
            bufferIndex = 0; // Reset for next command
        } else if (bufferIndex < 63) {
            inputBuffer[bufferIndex++] = c;
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

