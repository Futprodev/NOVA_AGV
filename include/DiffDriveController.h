#pragma once
#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "Odometry.h"

struct WheelState {
  float vRef; // m/s reference
  float vMeas; // m/s measured
  float prevErr;
  int pwmCmd;
};

class DiffDriveController {
  public:
    DiffDriveController(MotorDriver &left, MotorDriver &right, OdometryState &odom)
    : _left(left),
      _right(right),
      _odom(odom),
      _moving(false),
      _targetDistance(0.0f),
      _vCmd(0.0f),
      _vMax(0.1f),   // default, can be overridden
      _aAcc(0.1f),
      _aDec(0.1f),
      _yaw(0.0f),
      _yawStart(0.0f),
      _useHeading(false)
    {
      _wheelL = {0.0f, 0.0f, 0.0f, 0};
      _wheelR = {0.0f, 0.0f, 0.0f, 0};
    }

    void init() {
      _left.setRun(false);
      _right.setRun(false);
      delay(2500);
      _left.setDirection(true);
      _right.setDirection(true);  
    }

    void setYaw(float yawRad) {
      _yaw = yawRad;
    }

    void setProfileParams(float vMax, float aAcc, float aDec) {
      _vMax = vMax;
      _aAcc = aAcc;
      _aDec = aDec;
    }

    void startMove(float distance_m, float vMax_mps) {
      _targetDistance = distance_m;
      _odom.totalDistance = 0.0f;

      _vCmd          = 0.0f;
      _vMax          = vMax_mps;
      _wheelL.vRef   = 0.0f;
      _wheelR.vRef   = 0.0f;
      _wheelL.vMeas  = 0.0f;
      _wheelR.vMeas  = 0.0f;
      _wheelL.prevErr = 0.0f;
      _wheelR.prevErr = 0.0f;
      _wheelL.pwmCmd  = 0;
      _wheelR.pwmCmd  = 0;
      _left.setPWM(0);
      _right.setPWM(0);

      // enable drivers
      _left.setRun(true);
      _right.setRun(true);

      _yawStart   = _yaw;
      _useHeading = true;

      _moving = true;
      Serial.print("Starting move: ");
      Serial.print(distance_m);
      Serial.println(" m");
    } 
    
    bool isMoving() const { return _moving; }

    void setMeasuredSpeeds (float vL_meas, float vR_meas) {
      _wheelL.vMeas = vL_meas;
      _wheelR.vMeas = vR_meas;
    }

    // Direct wheel-velocity control (used for rotation)
    void setWheelRefs(float vL_ref, float vR_ref) {
      _wheelL.vRef = vL_ref;
      _wheelR.vRef = vR_ref;
    }

    // Run only the PD on current refs (no trapezoid / distance logic)
    void updatePD(float dt) {
      updateWheelPD(_wheelL, _left, dt);
      updateWheelPD(_wheelR, _right, dt);
    }

    void update(float dt) {
      if(_moving) {
        float remaining = _targetDistance - _odom.totalDistance;
        if (remaining <= 0.0f) {
          stopNow();
        } else {
          // velocity profile
          float v_meas = 0.5f * (fabs(_wheelL.vMeas) + fabs(_wheelR.vMeas));
          float d_brake = (v_meas * v_meas) / (2.0f * _aDec);
          float margin = 0.01f;

          if (remaining > d_brake + margin) {
            _vCmd += _aAcc * dt;
            
            if (_vCmd > _vMax) _vCmd = _vMax;
          } else {
            _vCmd -= _aDec * dt;
            if (_vCmd < 0.0f) _vCmd = 0.0f;
          }

          float vL_ref = _vCmd;
          float vR_ref = _vCmd;

          if (_useHeading && _vCmd > 0.05f) {
            float yawErr = _yaw - _yawStart;
            float w_corr = -K_HEADING * yawErr;
            float halfB = BASE_WIDTH * 0.5f;

            vL_ref = _vCmd - w_corr * halfB;
            vR_ref = _vCmd + w_corr * halfB;
          }

            _wheelL.vRef = vL_ref;
            _wheelR.vRef = vR_ref;
          }
        }
      
      if (_moving) {
        updateWheelPD(_wheelL, _left, dt);
        updateWheelPD(_wheelR, _right, dt);
      }
      
    }

  const WheelState& leftWheel()  const { return _wheelL; }
  const WheelState& rightWheel() const { return _wheelR; }

  private:
    MotorDriver &_left;
    MotorDriver &_right;
    OdometryState &_odom;

    WheelState _wheelL;
    WheelState _wheelR;

    bool  _moving;
    float _targetDistance;

    float _vCmd;
    float _vMax;
    float _aAcc;
    float _aDec;
    float _yaw;        
    float _yawStart;   
    bool  _useHeading; 

    void updateWheelPD(WheelState &w, MotorDriver &motor, float dt) {
      // 1) Split into sign + magnitude
      float vRef   = w.vRef;
      float vMeas  = w.vMeas;   // from odometry (always ≥ 0 in your case)

      bool  forward   = (vRef >= 0.0f);
      float vRefMag   = fabs(vRef);
      float vMeasMag  = fabs(vMeas);

      // 2) PD on magnitudes
      float err  = vRefMag - vMeasMag;
      float derr = (err - w.prevErr) / dt;
      w.prevErr  = err;

      float du = Kp_SPEED * err + Kd_SPEED * derr;

      w.pwmCmd += (int)du;
      w.pwmCmd = clampValue<int>(w.pwmCmd, 0, PWM_MAX);

      // 3) Apply direction + PWM
      motor.setDirection(forward);
      motor.setRun(true);
      motor.setPWM(w.pwmCmd);
    }


    void stopNow() {
      _wheelL.vRef = 0.0f;
      _wheelR.vRef = 0.0f;
      _wheelL.pwmCmd = 0;
      _wheelR.pwmCmd = 0;
      _left.setPWM(0);
      _right.setPWM(0);

      _left.setRun(false);
      _right.setRun(false);
      _moving = false;
      Serial.println("Target distance reached, stopping.");
    }
};