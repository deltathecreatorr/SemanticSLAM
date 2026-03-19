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
    gpio_handle = lgGpiochipOpen(4); // Pi 5 RP1 Chip
    if (gpio_handle < 0) exit(1);

    int pins[] = {FL_IN1, FL_IN2, FR_IN1, FR_IN2, BL_IN1, BL_IN2, BR_IN1, BR_IN2};
    for(int p : pins) {
        lgGpioClaimOutput(gpio_handle, 0, p, 0);
    }
}

void move(int state) {
    // Left side Forward
    lgGpioWrite(gpio_handle, FL_IN1, state); lgGpioWrite(gpio_handle, FL_IN2, 0);
    lgGpioWrite(gpio_handle, BL_IN1, state); lgGpioWrite(gpio_handle, BL_IN2, 0);
    // Right side Forward
    lgGpioWrite(gpio_handle, FR_IN1, state); lgGpioWrite(gpio_handle, FR_IN2, 0);
    lgGpioWrite(gpio_handle, BR_IN1, state); lgGpioWrite(gpio_handle, BR_IN2, 0);
}

int main() {
    setup();
    fmt::print("4WD Test Starting...\n");
    
    move(1); 
    std::this_thread::sleep_for(std::chrono::seconds(2));
    move(0); 

    fmt::print("4WD Test sequence complete. Service staying alive for commands...\n");

    // KEEP THE SERVICE ALIVE
    while(true) {
        // This loop prevents the program from exiting.
        // In the future, this is where your SLAM logic or 
        // ROS2 node spinning will happen.
        std::this_thread::sleep_for(std::chrono::hours(1));
    }

    lgGpiochipClose(gpio_handle);
    return 0;
}