/**
 * @file power_monitor.h
 * @brief INA219 + high-side switch control
 */
#pragma once
#include <Adafruit_INA219.h>

class PowerMonitor {
public:
  void begin();
  void update();                     // Call every loop or every 100ms
  void enableDriveRail(bool on);
  void enableArmRail(bool on);

  float getBusVoltage_V();
  float getCurrent_mA();
  float getPower_mW();

  bool isDriveEnabled() const { return driveEnabled; }
  bool isArmEnabled() const   { return armEnabled; }

private:
  Adafruit_INA219 ina219 = Adafruit_INA219(INA219_ADDRESS);

  bool driveEnabled = false;
  bool armEnabled   = false;
};