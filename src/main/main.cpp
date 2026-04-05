#include <iostream>
#include <thread>
#include <chrono>

#include "movementScripts/movementScripts.hpp"
#include "sensor_pkgs/lidar_node/lidar_node.hpp"
#include "sensor_pkgs/camera_pkg/camera_node.hpp"

void LidarNodeThread(int argc, char * argv[]) {
    // Lidar node setup
    std::cout << "Starting Lidar Node..." << std::endl;
    rclcpp::init(argc, argv);
    auto lidar_node = std::make_shared<LidarNode>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(lidar_node);
    std::thread spin_thread([&executor]() {
        executor.spin();
    });

    // Camera node setup
    auto camera_node = std::make_shared<CameraNode>();
    executor.add_node(camera_node);

    while (rclcpp::ok()) {
        if (lidar_node->data_received_) {
            std::cout << "Closest Point - X: "
                      << lidar_node->current_x_ << " Y: "
                      << lidar_node->current_y_ << std::endl;
        }

        if (camera_node->frame_received_) {
            std::cout << "Received Image - Width: "
                      << camera_node->image_width_ << " Height: "
                      << camera_node->image_height_ << " Encoding: "
                      << camera_node->encoding_ << std::endl;
        }



        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rclcpp::shutdown();
    spin_thread.join();
}

int main(int argc, char * argv[]) {
    LidarNodeThread(argc, argv);
    return 0;
}
