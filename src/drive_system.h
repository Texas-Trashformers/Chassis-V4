#pragma once
#include <Arduino.h>
#include "config.h"

class DriveSystem {
public:
  void begin();
  void update(const InputState& input);

private:
  int currentSpeedLimit = 150; // Default PWM speed (0-255)
  bool fastMode = false;

  // Helper to set individual TB6612 motor channels
  void setMotor(int pwmPin, int in1Pin, int in2Pin, int speed);
  
  // Helper to convert joystick enum to a numeric vector (-1, 0, or 1)
  void getJoystickVector(joystickpos pos, int &x, int &y);
};