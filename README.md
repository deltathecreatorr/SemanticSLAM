# Semantic Simultaneous Location and Mapping (SLAM)

This repository holds the code for the semantic SLAM algorithm created for a custom made robot that creates a semantic map using a 2D LiDAR and odometry sensors.

## Description

This project is dedicated to building a SLAM algorithm that utilises data from a 2D LiDAR sensor and odometry sensors. The LiDAR provides the outline of it's surroundings within a designated area. This data will be compiled to eventually create a floor plan for an entire room. The odometry sensors will track the movements of the robot within the environment, allowing the robot to localise itself within the environment, and gauge the distance between objects that are detected within a room. This will create the entire SLAM algorithm.

The SLAM algorithm will be complimented by a deep learning model that will classify objects it sees from a live camera feed. The camera module used is able to run a CNN on it, allowing the main system to bear the overhead of running a deep learning model at the same time. The deep learning model is trained on publicly found datasets of household objects, as the main environment this robot will be used in is a household.

## Building

This project can be built on any operating system supported by CMake.

Clone the repository and please install the CMake compiler before trying to build this project.

Afterwards, use the commands:

```cmake --build build```
```.build\Debug\slam.exe```

## Robot Car Parts

* [Raspberry Pi 5 8GB](https://thepihut.com/collections/raspberry-pi)
* [Mecanum MP Robot Chassis Kit](https://www.waveshare.com/robot-chassis.htm)
* [Raspberry Pi AI Camera Module](https://thepihut.com/products/raspberry-pi-ai-camera)
* [Uninterruptible Power Supply UPS HAT (B) for Raspberry Pi](https://thepihut.com/products/uninterruptible-power-supply-ups-hat-b-for-raspberry-pi)

## License

This project is licensed under the MiT License - see the LICENSE.md file for details.

