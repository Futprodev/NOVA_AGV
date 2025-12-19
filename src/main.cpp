#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Odometry.h"
#include "DiffDriveController.h"
#include "Imu.h"

// ----------------- wrap angle helper --------------------
static float wrapAngle(float a) {
  while (a >  M_PI) a -= 2.0f * M_PI;
  while (a < -M_PI) a += 2.0f * M_PI;
  return a;
}

// ---------------- Constants & globals --------------------
Imu6050 imu;

// polarity checked in MotorDriver 
MotorDriver leftMotor (PIN_PWM_L, PIN_DIR_L, PIN_STOP_L, true);
MotorDriver rightMotor(PIN_PWM_R, PIN_DIR_R, PIN_STOP_R, false);

OdometryState odom;
DiffDriveController controller(leftMotor, rightMotor, odom);

// Hall interrupts
volatile long g_hallCountL = 0;
volatile long g_hallCountR = 0;

void hallL_isr() { g_hallCountL++; }
void hallR_isr() { g_hallCountR++; }

// Timing / odom scale
uint32_t lastLoop       = 0;
uint32_t lastOdomSend   = 0;
float    distPerTick    = 0.0f;

const uint16_t ODOM_PERIOD_MS = 50;   // send O: line every 50 ms

// ---------------- Commanded velocities --------------------

float cmdV = 0.0f;     // linear  m/s
float cmdW = 0.0f;     // angular rad/s

unsigned long lastCmdTime = 0;
const unsigned long CMD_TIMEOUT_MS = 500; // stop if no command for 500 ms

// ---------------- stop helper   --------------------

void stopAllMotors() {
  controller.setWheelRefs(0.0f, 0.0f);
  controller.updatePD(0.01f);   // small dt to drive PWM -> 0
  leftMotor.setRun(false);
  rightMotor.setRun(false);
}

// ---------------- Serial command handling  --------------------
//
// Expect lines like:
//   V:0.20 W:0.00
//   V:-0.15 W:0.20
// Whitespace is allowed between tokens.
//
static char    lineBuf[64];
static uint8_t lineIdx = 0;

void handleLine(char *s) {
  bool got = false;

  char *vPtr = strstr(s, "V:");
  char *wPtr = strstr(s, "W:");

  if (vPtr) { cmdV = atof(vPtr + 2); got = true; }
  if (wPtr) { cmdW = atof(wPtr + 2); got = true; }

  if (got) {
    lastCmdTime = millis();
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (lineIdx == 0) continue; // ignore empty lines

      lineBuf[lineIdx] = '\0';
      lineIdx = 0;

      handleLine(lineBuf);
    } else {
      if (lineIdx < sizeof(lineBuf) - 1) {
        lineBuf[lineIdx++] = c;
      }
    }
  }
}

// -------------- setup -------------

void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("NOVA AGV Mega serial bridge starting...");
  Serial.println("Protocol:");
  Serial.println("  PC -> Mega: V:<v> W:<w>");
  Serial.println("  Mega -> PC: O:<x> <y> <theta> <v> <w>");

  // Motors & odom
  leftMotor.begin();
  rightMotor.begin();

  initOdometry(odom);

  controller.init();                 // sets logical directions, stops motors
  controller.setProfileParams(0.3f, 0.2f, 0.2f); // not used for cmd_vel, OK

  // IMU
  imu.begin();

  // Halls
  pinMode(PIN_HALL_L, INPUT);
  pinMode(PIN_HALL_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_L), hallL_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_HALL_R), hallR_isr, RISING);

  // Odometry scale
  distPerTick = 2.0f * PI * WHEEL_RADIUS / (float)TICKS_PER_REV;
  Serial.print("distPerTick = ");
  Serial.print(distPerTick, 6);
  Serial.println(" m");

  lastLoop     = millis();
  lastOdomSend = millis();
  lastCmdTime  = millis();
}

// -------------- loop -------------

void loop() {
  // 1) Read incoming V:.. W:.. lines from Pi
  handleSerial();

  uint32_t now = millis();
  if (now - lastLoop < CONTROL_PERIOD_MS) {
    return;
  }

  float dt = (now - lastLoop) / 1000.0f;
  lastLoop = now;

  // 2) Command timeout: stop if no new command for too long
  if ((now - lastCmdTime) > CMD_TIMEOUT_MS) {
    cmdV = 0.0f;
    cmdW = 0.0f;
    stopAllMotors();
    return;
  }

  // 3) Update IMU
  imu.update(dt);
  float yawIMU = imu.yawRad(); // you can choose later whether to fuse this into odom.theta

  // 4) Read halls & odometry
  long cL, cR;
  noInterrupts();
  cL = g_hallCountL;
  cR = g_hallCountR;
  g_hallCountL = 0;
  g_hallCountR = 0;
  interrupts();

  float dL = cL * distPerTick;
  float dR = cR * distPerTick;

  float dir = (cmdV >= 0.0f) ? 1.0f : -1.0f;
  float dL_signed = dir * dL;
  float dR_signed = dir * dR;

  float vL_meas = (dt > 0) ? dL_signed / dt : 0.0f;
  float vR_meas = (dt > 0) ? dR_signed / dt : 0.0f;

  updateOdometry(odom, dL_signed, dR_signed, BASE_WIDTH);

  // 5) Convert (v, w) -> wheel refs and run PD
  float halfB = BASE_WIDTH * 0.5f;
  float vL_ref = cmdV - cmdW * halfB;
  float vR_ref = cmdV + cmdW * halfB;

  leftMotor.setRun(true);
  rightMotor.setRun(true);

  controller.setWheelRefs(vL_ref, vR_ref);
  controller.setMeasuredSpeeds(vL_meas, vR_meas);
  controller.updatePD(dt);

  // 6) Periodically send odometry back to Pi
  if (now - lastOdomSend >= ODOM_PERIOD_MS) {
    lastOdomSend = now;

    // For now, use odom.theta; later you can blend with yawIMU
    Serial.print("O:");
    Serial.print(odom.x, 3);     Serial.print(' ');
    Serial.print(odom.y, 3);     Serial.print(' ');
    Serial.print(odom.theta, 3); Serial.print(' ');
    Serial.print(cmdV, 3);       Serial.print(' ');
    Serial.print(cmdW, 3);
    Serial.println();
  }
}
