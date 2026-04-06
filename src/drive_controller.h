#pragma once
#include "config.h"
#include "power_monitoring.h"
#include "motor_driver.h"

class DriveController {
public:
  void begin();
  void update(const InputState& pkt);

  PowerMonitor power;
  MotorDriver motors;
};
