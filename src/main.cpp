#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Odometry.h"
#include "DiffDriveController.h"
#include "Imu.h"

Imu6050 imu;

// ISR for counting hall sensors
volatile long g_hallCountL = 0;
volatile long g_hallCountR = 0;

void hallL_isr() {
  g_hallCountL++;
}

void hallR_isr() {
  g_hallCountR++;
}

// Motor driver and odom
MotorDriver leftMotor (PIN_PWM_L, PIN_DIR_L, PIN_STOP_L);   // inverted
MotorDriver rightMotor(PIN_PWM_R, PIN_DIR_R, PIN_STOP_R);  // normal

OdometryState odom;
DiffDriveController controller(leftMotor, rightMotor, odom);

// time
uint32_t lastLoop = 0;

// distance
float distPerTick = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(" Test Start ");

  leftMotor.begin();
  rightMotor.begin();

  initOdometry(odom);

  controller.init();
  controller.setProfileParams(0.5, 0.2, 0.2);

  imu.begin();

  pinMode(PIN_HALL_L, INPUT);
  pinMode(PIN_HALL_R, INPUT);

  // pin 2/3
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_L), hallL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_R), hallR_isr, RISING);

  // distance per tick
  distPerTick = 2.0f * PI * WHEEL_RADIUS / (float)TICKS_PER_REV;
  Serial.print("distPerTick = ");
  Serial.print(distPerTick, 6);
  Serial.println(" m");

  controller.startMove(10.0f, 0.5f);

  lastLoop = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastLoop < CONTROL_PERIOD_MS) {
    return;
  }

  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  imu.update(dt);
  float yaw = imu.yawRad();

  // reset and read hall counts
  long cL, cR;
  noInterrupts();
  cL = g_hallCountL;
  cR = g_hallCountR;
  g_hallCountL = 0;
  g_hallCountR = 0;
  interrupts();

  // compute distance
  float dL = cL * distPerTick;
  float dR = cR * distPerTick;

  float vL_meas = dL/dt;
  float vR_meas = dR/dt;

  updateOdometry(odom, dL, dR, BASE_WIDTH);

  controller.setMeasuredSpeeds(vL_meas, vR_meas);
  controller.update(dt);

  controller.setYaw(yaw);              
  controller.setMeasuredSpeeds(vL_meas, vR_meas);
  controller.update(dt);

  const WheelState &wL = controller.leftWheel();
  const WheelState &wR = controller.rightWheel();

  Serial.print("vL: "); Serial.print(vL_meas, 3);
  Serial.print("  vR: "); Serial.print(vR_meas, 3);
  Serial.print("  pwmL: "); Serial.print(wL.pwmCmd);
  Serial.print("  pwmR: "); Serial.print(wR.pwmCmd);
  Serial.print("  x: "); Serial.print(odom.x, 3);
  Serial.print("  y: "); Serial.print(odom.y, 3);
  Serial.print("  th: "); Serial.print(odom.theta, 3);
  Serial.print("  dist: "); Serial.print(odom.totalDistance, 3);
  Serial.print("  yawIMU: "); Serial.print(yaw, 3);
  Serial.print("  dist: "); Serial.print(odom.totalDistance, 3);
  Serial.println();
}