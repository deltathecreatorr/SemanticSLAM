#include <iostream>
#include <thread>
#include <chrono>
#

#include "movementScripts/movementScripts.h"

int main() {
    setup();

    std::cout << "Moving Forward..." << std::endl;
    moveForward(5);

    allStop(5);

    std::cout << "Moving Backward..." << std::endl;
    moveBackward(5);

    allStop(5);
    std::cout << "Done." << std::endl;

    return 0;
}
