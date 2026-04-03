#pragma once
#include "config.h"
#include "power_monitoring.h"     // ← note the filename

class DriveController {
public:
  void begin();
  void update(const InputState& pkt);

  PowerMonitor power;
};