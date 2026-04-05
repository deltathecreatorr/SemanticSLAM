#include "movementScripts.hpp"
#include <iostream>
#include <lgpio.h>
#include <fmt/core.h>
#include <thread>

const int BL_IN1 = 27; const int BL_IN2 = 17;
const int FL_IN1 = 22; const int FL_IN2 = 23;
const int FR_IN1 = 25; const int FR_IN2 = 24;
const int BR_IN1 = 7;  const int BR_IN2 = 8;

int gpio_handle = -1;

void setup() {
    // Try chip 4 first (Standard RP1), if it fails, try 0
    gpio_handle = lgGpiochipOpen(4); 
    if (gpio_handle < 0) {
        fmt::print("Failed to open gpiochip 4, trying chip 0...\n");
        gpio_handle = lgGpiochipOpen(0);
    }
    
    if (gpio_handle < 0) {
        fmt::print("Critical Error: Could not open any GPIO chip.\n");
        exit(1);
    }

    int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2};
    for(int p : pins) {
        lgGpioClaimOutput(gpio_handle, 0, p, 0);
    }
}

void allStop(int seconds) {
    int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2};
    for(int p : pins) lgGpioWrite(gpio_handle, p, 0);
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
}

int moveForward(int seconds) {
    // Set all motors to move forward
    lgGpioWrite(gpio_handle, FL_IN1, 1); lgGpioWrite(gpio_handle, FL_IN2, 0);
    lgGpioWrite(gpio_handle, FR_IN1, 1); lgGpioWrite(gpio_handle, FR_IN2, 0);
    lgGpioWrite(gpio_handle, BL_IN1, 1); lgGpioWrite(gpio_handle, BL_IN2, 0);
    lgGpioWrite(gpio_handle, BR_IN1, 1); lgGpioWrite(gpio_handle, BR_IN2, 0);
    
    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return 0; // Return status code if needed
}

int moveBackward(int seconds) {
    // Set all motors to move backward
    lgGpioWrite(gpio_handle, FL_IN1, 0); lgGpioWrite(gpio_handle, FL_IN2, 1);
    lgGpioWrite(gpio_handle, FR_IN1, 0); lgGpioWrite(gpio_handle, FR_IN2, 1);
    lgGpioWrite(gpio_handle, BL_IN1, 0); lgGpioWrite(gpio_handle, BL_IN2, 1);
    lgGpioWrite(gpio_handle, BR_IN1, 0); lgGpioWrite(gpio_handle, BR_IN2, 1);

    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return 0; // Return status code if needed
}

int rotateRight(int seconds) {
    // Set left motors forward and right motors backward for rotation
    lgGpioWrite(gpio_handle, FL_IN1, 1); lgGpioWrite(gpio_handle, FL_IN2, 0);
    lgGpioWrite(gpio_handle, FR_IN1, 0); lgGpioWrite(gpio_handle, FR_IN2, 1);
    lgGpioWrite(gpio_handle, BL_IN1, 1); lgGpioWrite(gpio_handle, BL_IN2, 0);
    lgGpioWrite(gpio_handle, BR_IN1, 0); lgGpioWrite(gpio_handle, BR_IN2, 1);

    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return 0; // Return status code if needed
}

int rotateLeft(int seconds) {
    // Set right motors forward and left motors backward for rotation
    lgGpioWrite(gpio_handle, FL_IN1, 0); lgGpioWrite(gpio_handle, FL_IN2, 1);
    lgGpioWrite(gpio_handle, FR_IN1, 1); lgGpioWrite(gpio_handle, FR_IN2, 0);
    lgGpioWrite(gpio_handle, BL_IN1, 0); lgGpioWrite(gpio_handle, BL_IN2, 1);
    lgGpioWrite(gpio_handle, BR_IN1, 1); lgGpioWrite(gpio_handle, BR_IN2, 0);

    std::this_thread::sleep_for(std::chrono::seconds(seconds));
    return 0; // Return status code if needed
}