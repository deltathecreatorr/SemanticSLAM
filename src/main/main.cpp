#include <iostream>
#include <thread>
#include <chrono>

#include "movementScripts/movementScripts.h"

int main() {
    setup();

    std::cout << "Moving Forward..." << std::endl;
    moveForward(2);

    allStop(2);

    std::cout << "Moving Backward..." << std::endl;
    moveBackward(2);

    allStop(2);
    std::cout << "Done." << std::endl;

    return 0;
}
