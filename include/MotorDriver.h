#pragma once
#include <Arduino.h>
#include "Config.h"

class MotorDriver {
  public:
    MotorDriver(int pwmPin, int dirPin, int stopPin)
    : _pwmPin(pwmPin),
      _dirPin(dirPin),
      _stopPin(stopPin),
      _pwmCmd(0),
      _forward(true)
    {}

    void begin() {
      if (_dirPin  >= 0) pinMode(_dirPin, OUTPUT);
      if (_stopPin >= 0) pinMode(_stopPin, OUTPUT);
      pinMode(_pwmPin, OUTPUT);

      // Default: enable driver & set some direction
      setRun(false);        // STOP = LOW (run) for your board
      setDirection(true);  // logical "forward" = DIR LOW (we'll define that)
      setPWM(0);
    }

    // forward = true  -> DIR LOW
    // forward = false -> DIR HIGH
    void setDirection(bool forward) {
      _forward = forward;
      if (_dirPin >= 0) {
        digitalWrite(_dirPin, forward ? LOW : HIGH);
      }
    }

    // enable = true  -> STOP LOW  (run)
    // enable = false -> STOP HIGH (stop)
    void setRun(bool enable) {
      if (_stopPin >= 0) {
        digitalWrite(_stopPin, enable ? LOW : HIGH);
      }
    }

    void setPWM(int value) {
      _pwmCmd = clampValue<int>(value, 0, PWM_MAX);
      analogWrite(_pwmPin, _pwmCmd);
    }

    int  getPWM()    const { return _pwmCmd; }
    bool isForward() const { return _forward; }

  private:
    int  _pwmPin;
    int  _dirPin;
    int  _stopPin;
    int  _pwmCmd;
    bool _forward;
};
