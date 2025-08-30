
#pragma once

#include <stdint.h>
#include "utilities.hpp"
#include "i2cSensorClass.hpp"

#define BNO055_CHIP_ID  0x00
#define BNO055_CHIP_ID_VALUE  0xA0

#define BNO055_EULER_DATA_ID 0x1A
#define BNO055_QUAT_DATA_ID 0x20
#define BNO055_LIN_ACCEL_DATA_ID 0x28
#define BNO055_GRAVITY_DATA_ID 0x2E

#define BNO055_CALIB_REG_NAME 0x35
#define BNO055_OPR_REG_NAME 0x3D
#define BNO055_PWR_REG_NAME 0x3E
#define BNO055_SYS_TRIGGER 0x3F

class bno055SensorClass: public i2cSensorClass {

private:
  uint8_t sensorAddress = 0;
  uint8_t powerMode = 0xFF;
  uint8_t configMode = 0x00;
  uint8_t calibStatus = 0;

public:
  bno055SensorClass(uint8_t address): i2cSensorClass(address) {
    // Verify Chip ID
    verifyDeviceID(BNO055_CHIP_ID);
    ESP_LOGI(TAG, "Device Verified\n");

    // Soft reset
    writeRegister(BNO055_SYS_TRIGGER, 0x20);
    vTaskDelay(pdMS_TO_TICKS(700)); // Should wait for BNO055 device to startup after reset (~650ms)
    ESP_LOGI(TAG, "Device RESET\n");
    
    // Default power mode = Normal Mode
    powerMode = 0x00;
    writeRegister(BNO055_PWR_REG_NAME, powerMode);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Device Power Mode Set\n");
    
    // Default config = NDOF Mode
    configMode = 0x0C;
    writeRegister(BNO055_OPR_REG_NAME, configMode);
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "Device Configuration complete\n");
    ESP_LOGI(TAG, "BNO055 device setup successful\n");
  }

  void verifyDeviceID(uint8_t chipID = 0) {
    readRegister(chipID);
    if (getDataPtr()[0] != BNO055_CHIP_ID_VALUE)
      ESP_LOGE(TAG, "I2C device verification failed\n");
  }

  // Get orientation in Euler angles
  void getEulerAngles(float& yaw, float& roll, float& pitch) {
    uint8_t* commDataRegPtr = getDataPtr();
    readRegister(BNO055_EULER_DATA_ID, 6);

    int16_t tempYaw = (commDataRegPtr[0] | (commDataRegPtr[1] << 8));
    int16_t tempRoll = (commDataRegPtr[2] | (commDataRegPtr[3] << 8));
    int16_t tempPitch = (commDataRegPtr[4] | (commDataRegPtr[5] << 8));

    yaw = tempYaw / 16.f;
    roll = tempRoll / 16.f;
    pitch = tempPitch / 16.f;
  }

  // Get orientation in quaternions
  void getQuatAngles(float& w, float& x, float& y, float& z) {
    const uint8_t* commDataRegPtr = getDataPtr();
    readRegister(BNO055_QUAT_DATA_ID, 8);

    int16_t tempW = (commDataRegPtr[0] | (commDataRegPtr[1] << 8));
    int16_t tempX = (commDataRegPtr[2] | (commDataRegPtr[3] << 8));
    int16_t tempY = (commDataRegPtr[4] | (commDataRegPtr[5] << 8));
    int16_t tempZ = (commDataRegPtr[6] | (commDataRegPtr[7] << 8));

    w = tempW / 16.f;
    x = tempX / 16.f;
    y = tempY / 16.f;
    z = tempZ / 16.f;
  }

  // Get Linear Acceleration Values
  void getLinAccel(float& x, float& y, float& z) {
    const uint8_t* commDataRegPtr = getDataPtr();
    readRegister(BNO055_LIN_ACCEL_DATA_ID, 6);

    int16_t tempX = (commDataRegPtr[0] | (commDataRegPtr[1] << 8));
    int16_t tempY = (commDataRegPtr[2] | (commDataRegPtr[3] << 8));
    int16_t tempZ = (commDataRegPtr[4] | (commDataRegPtr[5] << 8));

    x = tempX / 16.f;
    y = tempY / 16.f;
    z = tempZ / 16.f;
  }

  // Get Gravity Values
  void getGravity(float& x, float& y, float& z) {
    const uint8_t* commDataRegPtr = getDataPtr();
    readRegister(BNO055_GRAVITY_DATA_ID, 6);

    int16_t tempX = (commDataRegPtr[0] | (commDataRegPtr[1] << 8));
    int16_t tempY = (commDataRegPtr[2] | (commDataRegPtr[3] << 8));
    int16_t tempZ = (commDataRegPtr[4] | (commDataRegPtr[5] << 8));

    x = tempX / 16.f;
    y = tempY / 16.f;
    z = tempZ / 16.f;
  }

  // Get Calibration Data
  bool isSensorCalibrated() {
    const uint8_t* commDataRegPtr = getDataPtr();
    readRegister(BNO055_CALIB_REG_NAME, 1);
    uint8_t calibValue = commDataRegPtr[0];

    uint8_t sysCalib = (calibValue >> 6) & 0x03;
    uint8_t gyroCalib = (calibValue >> 4) & 0x03;
    uint8_t accelCalib = (calibValue >> 2) & 0x03;
    uint8_t magCalib = calibValue & 0x03;

    // All calib values should be equal to 3
    // sysCalib = 3, gyroCalib = 3, accelCalib = 3, magCalib = 3 for sensor to be calibrated
    return (sysCalib >= 2) && (gyroCalib > 2) && (accelCalib > 2) && (magCalib > 2);
  }
};