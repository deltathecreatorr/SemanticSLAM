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

void streamIMUData() {
    if (imu.readSensor()) {
        Serial.print("IMU,");
        Serial.print(imu.getAccelX_mss(), 4); Serial.print(",");
        Serial.print(imu.getAccelY_mss(), 4); Serial.print(",");
        Serial.print(imu.getAccelZ_mss(), 4); Serial.print(",");
        Serial.print(imu.getGyroX_rads(), 4); Serial.print(",");
        Serial.print(imu.getGyroY_rads(), 4); Serial.print(",");
        Serial.println(imu.getGyroZ_rads(), 4);
    }
}
