#pragma once
#include <Arduino.h>
#include "Config.h"

class MotorDriver {
  public:
    // polarity = false  -> normal wiring
    // polarity = true   -> flipped wiring (invert logical forward)
    MotorDriver(int pwmPin, int dirPin, int stopPin, bool polarity)
    : _pwmPin(pwmPin),
      _dirPin(dirPin),
      _stopPin(stopPin),
      _pwmCmd(0),
      _forward(true),
      _polarity(polarity)
    {}

    void begin() {
      if (_dirPin  >= 0) pinMode(_dirPin, OUTPUT);
      if (_stopPin >= 0) pinMode(_stopPin, OUTPUT);
      pinMode(_pwmPin, OUTPUT);

      // start stopped
      setRun(false);
      setDirection(true);   // logical forward
      setPWM(0);
    }

    // forward = true  -> logical forward
    // polarity         adjusts how that maps to pin level
    void setDirection(bool forward) {
      _forward = forward;

      if (_dirPin >= 0) {
        // XOR with polarity: if polarity is true, flip the meaning
        bool logicalForward = forward ^ _polarity;

        // keep your original LOW/HIGH mapping
        digitalWrite(_dirPin, logicalForward ? LOW : HIGH);
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
    bool _polarity;   // NEW
};
