#include <fmt/core.h>
#include <thread>
#include <chrono>
#include <lgpio.h>

// BCM Pin Definitions
const int FL_IN1 = 17; const int FL_IN2 = 27;
const int FR_IN1 = 22; const int FR_IN2 = 23;
const int BL_IN1 = 24; const int BL_IN2 = 25;
const int BR_IN1 = 8;  const int BR_IN2 = 7;

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

// Helper to stop everything
void allStop() {
    int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2};
    for(int p : pins) lgGpioWrite(gpio_handle, p, 0);
}

void testMotor(const std::string& name, int pin1, int pin2) {
    fmt::print("Testing: {} (Pins {} & {})\n", name, pin1, pin2);
    lgGpioWrite(gpio_handle, pin1, 1);
    lgGpioWrite(gpio_handle, pin2, 0);
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    lgGpioWrite(gpio_handle, pin1, 0);
    fmt::print("{} Stopped.\n", name);
    std::this_thread::sleep_for(std::chrono::milliseconds(500)); // Short pause between tests
}

int main() {
    setup();
    allStop(); // Ensure clean start

    fmt::print("--- Starting Individual Wheel Roll Call ---\n");
    
    testMotor("FRONT LEFT",  FL_IN1, FL_IN2);
    testMotor("FRONT RIGHT", FR_IN1, FR_IN2);
    testMotor("BACK LEFT",   BL_IN1, BL_IN2);
    testMotor("BACK RIGHT",  BR_IN1, BR_IN2);

    fmt::print("--- All tests complete. Keeping service alive. ---\n");

    while(true) {
        std::this_thread::sleep_for(std::chrono::hours(1));
    }

    lgGpiochipClose(gpio_handle);
    return 0;
}