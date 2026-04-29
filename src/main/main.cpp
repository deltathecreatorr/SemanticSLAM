#include <rclcpp/rclcpp.hpp>
#include "sensor_pkgs/pico_link/sensor_pkg/sensor_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    rclcpp::ExecutorOptions options;
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(options, 4);
    
    auto encoder_node = std::make_shared<EncoderNode>();

    executor->add_node(encoder_node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting the executor...");

    executor->spin();
    rclcpp::shutdown();
    return 0;
}