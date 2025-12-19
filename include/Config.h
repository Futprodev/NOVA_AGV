#pragma once
#include <Arduino.h>

// robot params
constexpr float WHEEL_RADIUS = 0.1016f;
constexpr float BASE_WIDTH = 0.35f;
constexpr float TICKS_PER_REV = 15;

// control loop
constexpr uint32_t CONTROL_PERIOD_MS = 50;

// pd gains
constexpr float Kp_SPEED = 20.0f;
constexpr float Kd_SPEED = 0.0f;
constexpr float K_HEADING = 3.0f;

// pins
constexpr int PIN_PWM_L  = 5;    // to module S (ANALOG)
constexpr int PIN_DIR_L  = 7;    // to DIR
constexpr int PIN_STOP_L = 8;    // to STOP

// Right motor
constexpr int PIN_PWM_R  = 6;    // to module S (ANALOG)
constexpr int PIN_DIR_R  = 9;

constexpr int PIN_STOP_R = 10;

// Hall 
constexpr int PIN_HALL_L = 2;
constexpr int PIN_HALL_R = 3;

// PWM (LEDC) config
constexpr int PWM_FREQ      = 20000;  // 20 kHz
constexpr int PWM_RES_BITS  = 10;     // 0..1023
constexpr int PWM_MAX       = 255;
constexpr int PWM_CH_L      = 0;
constexpr int PWM_CH_R      = 1;

// helper
template<typename T>
T clampValue(T v, T lo, T hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}
