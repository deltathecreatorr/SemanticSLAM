#include <fmt/core.h>
#include <slam/slam.h>
#include <thread>
#include <chrono>
#include <iostream>
#include <lgpio.h>

const int LEFT_IN1 = 17;
const int LEFT_IN2 = 27;

const int RIGHT_IN1 = 22;
const int RIGHT_IN2 = 23;

int gpio_handle = -1;

void setupMotors() {
    std::cout << "Initializing lgpio for Raspberry Pi 5..." << std::endl;

    gpio_handle = lgGpiochipOpen(0); 
    if (gpio_handle < 0) {
        gpio_handle = lgGpiochipOpen(4);
    }

    if (gpio_handle < 0) {
        std::cerr << "Failed to initialize GPIO chip. Are you running as root?" << std::endl;
        exit(1);
    }
    
    lgGpioClaimOutput(gpio_handle, 0, LEFT_IN1, 0);
    lgGpioClaimOutput(gpio_handle, 0, LEFT_IN2, 0);
    lgGpioClaimOutput(gpio_handle, 0, RIGHT_IN1, 0);
    lgGpioClaimOutput(gpio_handle, 0, RIGHT_IN2, 0);
}

void spinMotors() {
    fmt::print("Motors are spinning.\n");
    lgGpioWrite(gpio_handle, LEFT_IN1, 1);
    lgGpioWrite(gpio_handle, LEFT_IN2, 0);
    lgGpioWrite(gpio_handle, RIGHT_IN1, 1);
    lgGpioWrite(gpio_handle, RIGHT_IN2, 0);
}

void stopMotors() {
    fmt::print("Motors stopped.\n");
    lgGpioWrite(gpio_handle, LEFT_IN1, 0);
    lgGpioWrite(gpio_handle, LEFT_IN2, 0);
    lgGpioWrite(gpio_handle, RIGHT_IN1, 0);
    lgGpioWrite(gpio_handle, RIGHT_IN2, 0);
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

    lgGpiochipClose(gpio_handle);
    return 0;
}