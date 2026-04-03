/**
 * @file power_monitor.h
 * @brief INA219 + high-side switch control with heavy debug
 */
#pragma once
#include <Adafruit_INA219.h>
#include "config.h"

class PowerMonitor {
public:
  void begin();
  void update();                     // Call every loop (~10 ms)

  void enableDriveRail(bool on);
  void enableArmRail(bool on);

  float getBusVoltage_V()   { return ina219.getBusVoltage_V(); }
  float getCurrent_mA()     { return ina219.getCurrent_mA(); }
  float getPower_mW()       { return ina219.getPower_mW(); }

  bool isDriveEnabled() const { return driveEnabled; }
  bool isArmEnabled()   const { return armEnabled; }

private:
  Adafruit_INA219 ina219 = Adafruit_INA219(INA219_ADDRESS);

  bool driveEnabled = false;
  bool armEnabled   = false;
};