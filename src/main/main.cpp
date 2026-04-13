#include <rclcpp/rclcpp.hpp>

#include "sensor_pkgs/pico_link/pico_link.hpp"
#include "sensor_pkgs/pico_link/IMU_pkg/IMU_node.hpp"
#include "sensor_pkgs/pico_link/encoder_pkg/encoder_node.hpp"
#include "sensor_pkgs/pico_link/driver_pkg/driver_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    
    auto driver_node = std::make_shared<DriverNode>();

    auto pico_link = std::make_shared<PicoLink>();

    auto imu_node = std::make_shared<IMUNode>();

    auto encoder_node = std::make_shared<EncoderNode>();

    executor->add_node(driver_node);
    executor->add_node(pico_link);
    executor->add_node(imu_node);
    executor->add_node(encoder_node);

    RCLCPP_INFO(rclcpp::get_logger("main"), "Starting the executor...");

    executor->spin();
    rclcpp::shutdown();
    return 0;
}