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

// The pin we used to fix the short circuit!
const int OE_PIN = 12; 

int gpio_handle = -1;

void setupMotors() {
    std::cout << "Initializing lgpio for Raspberry Pi 5..." << std::endl;

    // Open the Pi 5 RP1 GPIO chip
    gpio_handle = lgGpiochipOpen(4);

    if (gpio_handle < 0) {
        std::cerr << "Failed to initialize GPIO chip. Are you running with sudo?" << std::endl;
        exit(1);
    }
    
    // Claim the motor pins
    lgGpioClaimOutput(gpio_handle, 0, LEFT_IN1, 0);
    lgGpioClaimOutput(gpio_handle, 0, LEFT_IN2, 0);
    lgGpioClaimOutput(gpio_handle, 0, RIGHT_IN1, 0);
    lgGpioClaimOutput(gpio_handle, 0, RIGHT_IN2, 0);

    // CLAIM AND WAKE UP THE LEVEL SHIFTER
    lgGpioClaimOutput(gpio_handle, 0, OE_PIN, 0);
    lgGpioWrite(gpio_handle, OE_PIN, 1); // Send 3.3V to OE to open the gates
    std::cout << "Level shifter awake!" << std::endl;
    
    // Give the chip 50 milliseconds to stabilize before sending motor commands
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

    // Spin for 2 seconds
    std::this_thread::sleep_for(std::chrono::seconds(2));

    stopMotors();

    // Clean up before exiting (Highly recommended so motors don't get stuck on!)
    lgGpiochipClose(gpio_handle);
    fmt::print("Test complete. Exiting.\n");
    
    return 0;
}