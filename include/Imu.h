#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050_light.h>

// Simple wrapper around MPU6050_light to give you yaw in radians
class Imu6050 {
public:
  Imu6050() : _mpu(Wire), _yawRad(0.0f) {}

  bool begin() {
    Wire.begin();           // SDA/SCL already defined for your board
    byte status = _mpu.begin();
    if (status != 0) {
      Serial.print("MPU6050 init failed, status = ");
      Serial.println(status);
      return false;
    }
    Serial.println("MPU6050 init OK, calibrating...");

    // Calibrate gyro/accel while robot is still
    delay(1000);
    _mpu.calcOffsets(true, true);  // gyro + accel
    Serial.println("MPU6050 calibration done.");
    _yawRad = 0.0f;
    return true;
  }

  // Call this every loop with dt [s]
  void update(float dt) {
    _mpu.update();
    float yawDeg = _mpu.getAngleZ(); // axis
    float yawRaw = yawDeg * PI / 180.0f;

    static float yawFilt = 0.0f;
    const float alpha = 0.1f;  // smaller = smoother
    yawFilt += alpha * (yawRaw - yawFilt);

    _yawRad = yawFilt;
  }

  float yawRad() const { return _yawRad; }

private:
  MPU6050 _mpu;
  float   _yawRad;
};
