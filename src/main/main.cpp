#include <fmt/core.h>
#include <slam/slam.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <pigpio.h>

const int LEFT_IN1 = 17;
const int LEFT_IN2 = 27;

const int RIGHT_IN1 = 22;
const int RIGHT_IN2 = 23;

void setupMotors() {
    fmt::print("Motors setup complete.\n");

    if (gpioInitialise() < 0) {
        fmt::print("Failed to initialize GPIO.\n");
        return;
    }

    gpioSetMode(LEFT_IN1, PI_OUTPUT);
    gpioSetMode(LEFT_IN2, PI_OUTPUT);
    gpioSetMode(RIGHT_IN1, PI_OUTPUT);
    gpioSetMode(RIGHT_IN2, PI_OUTPUT);
}

void spinMotors() {
    fmt::print("Motors are spinning.\n");
    gpioWrite(LEFT_IN1, 1);
    gpioWrite(LEFT_IN2, 0);
    gpioWrite(RIGHT_IN1, 1);
    gpioWrite(RIGHT_IN2, 0);
}

void stopMotors() {
    fmt::print("Motors stopped.\n");
    gpioWrite(LEFT_IN1, 0);
    gpioWrite(LEFT_IN2, 0);
    gpioWrite(RIGHT_IN1, 0);
    gpioWrite(RIGHT_IN2, 0);
}

int main() {
    fmt::print("Is this test for the CI/CD pipeline working rn?\n");
    setupMotors();
    spinMotors();

    std::this_thread::sleep_for(std::chrono::seconds(2));

    stopMotors();

    while(true) {
        std::this_thread::sleep_for(std::chrono::seconds(10));
    }

    gpioTerminate();
    return 0;
}