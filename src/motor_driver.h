#pragma once
#include "config.h"

class TB6612 {
public:
  void begin(uint8_t pwmPin, uint8_t in1, uint8_t in2, uint8_t invert = 0);
  void drive(int16_t speed);   // -255 to +255
  void brake();
  void coast();

private:
  uint8_t pwm, in1, in2, inv;
};

class MotorDriver {
public:
  TB6612 leftFront;
  TB6612 leftRear;
  TB6612 rightFront;
  TB6612 rightRear;

  void begin();
  void update(const InputState& pkt);

private:
  void handleDriveMode(const InputState& pkt);
  void handleArmMode(const InputState& pkt);
};