#include "IMUHandler.hpp"
#include <SPI.h>

const uint8_t IMU_CS_PIN = 17;

MPU9250 imu(SPI, IMU_CS_PIN);

bool initIMU() {
    int status = imu.begin();
    if (status < 0) {
        return false;
    }

    imu.setAccelRange(MPU9250::ACCEL_RANGE_4G);
    imu.setGyroRange(MPU9250::GYRO_RANGE_500DPS);

    return true;
}

bool populateIMUMsg(sensor_msgs__msg__Imu* msg) {
    if (imu.readSensor()) {
        msg->linear_acceleration.x = imu.getAccelX_mss();
        msg->linear_acceleration.y = imu.getAccelY_mss();
        msg->linear_acceleration.z = imu.getAccelZ_mss();
        
        msg->angular_velocity.x = imu.getGyroX_rads();
        msg->angular_velocity.y = imu.getGyroY_rads();
        msg->angular_velocity.z = imu.getGyroZ_rads() * -1.0; // The Z-flip!
        return true;
    }
    return false;
}
