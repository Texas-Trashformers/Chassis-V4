#include "power_monitoring.h"
#include "config.h"

void PowerMonitor::begin() {
  if (!ina219.begin()) {
    Serial.println("[POWER] INA219 init FAILED!");
  } else {
    Serial.println("[POWER] INA219 initialized");
  }

  // Start with both rails OFF (safe)
  pinMode(DRIVE_EN_PIN, OUTPUT);
  pinMode(ARM_EN_PIN, OUTPUT);
  enableDriveRail(false);
  enableArmRail(false);
}

void PowerMonitor::update() {
  float busV  = ina219.getBusVoltage_V();
  float shunt = ina219.getShuntVoltage_mV();
  float loadV = busV + (shunt / 1000.0);
  float current = ina219.getCurrent_mA();
  float power   = ina219.getPower_mW();

#ifdef SERIAL_DEBUG
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint > 500) {
    Serial.printf("[POWER] Bus: %.2fV  Load: %.2fV  Current: %.0fmA  Power: %.0fmW  | Drive:%s Arm:%s\n",
                  busV, loadV, current, power,
                  driveEnabled ? "ON" : "OFF",
                  armEnabled   ? "ON" : "OFF");
    lastPrint = millis();
  }
#endif
}

void PowerMonitor::enableDriveRail(bool on) {
  digitalWrite(DRIVE_EN_PIN, on ? HIGH : LOW);
  driveEnabled = on;
#ifdef SERIAL_DEBUG
  Serial.printf("[POWER] DRIVE rail %s\n", on ? "ENABLED" : "DISABLED");
#endif
}

void PowerMonitor::enableArmRail(bool on) {
  digitalWrite(ARM_EN_PIN, on ? HIGH : LOW);
  armEnabled = on;
#ifdef SERIAL_DEBUG
  Serial.printf("[POWER] ARM rail %s\n", on ? "ENABLED" : "DISABLED");
#endif
}

float PowerMonitor::getBusVoltage_V() { return ina219.getBusVoltage_V(); }
float PowerMonitor::getCurrent_mA()   { return ina219.getCurrent_mA(); }
float PowerMonitor::getPower_mW()     { return ina219.getPower_mW(); }