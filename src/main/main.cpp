#include <iostream>
#include <thread>
#include <chrono>

#include "movementScripts/movementScripts.h"

int main() {
    std::cout << "Starting robot control program..." << std::endl;
    setup();
    
    // Example movement sequence
    std::cout << "Moving forward..." << std::endl;
    // Code to move forward would go here
    moveForward();

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate movement duration

    std::cout << "Moving backward..." << std::endl;
    // Code to move backward would go here
    moveBackward();

    std::this_thread::sleep_for(std::chrono::seconds(2)); // Simulate movement duration

    std::cout << "Stopping..." << std::endl;
    allStop();

    return 0;
}
