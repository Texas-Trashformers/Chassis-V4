#include "power_monitoring.h"

void PowerMonitor::begin() {
  Serial.println("[POWER] Initializing INA219 (10 mΩ shunt)...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!ina219.begin()) {
    Serial.println("[POWER] ❌ INA219 init FAILED! (Check BATT rail power)");
  } else {
    ina219.setCalibration_16V_400mA();
    Serial.println("[POWER] ✅ INA219 ready (16V / 400 mA range)");
  }

  pinMode(DRIVE_EN_PIN, OUTPUT);
  pinMode(ARM_EN_PIN, OUTPUT);
  enableDriveRail(false);
  enableArmRail(false);
}

void PowerMonitor::update() {
  float busV    = ina219.getBusVoltage_V();
  float shuntV  = ina219.getShuntVoltage_mV();
  float loadV   = busV + (shuntV / 1000.0);
  float current = ina219.getCurrent_mA();
  float power   = ina219.getPower_mW();

#ifdef SERIAL_DEBUG
  static unsigned long lastPrint = 0;
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
  if (on == driveEnabled) return;        // ← only act if state actually changed
  digitalWrite(DRIVE_EN_PIN, on ? HIGH : LOW);
  driveEnabled = on;
#ifdef SERIAL_DEBUG
  Serial.printf("[POWER] DRIVE rail %s\n", on ? "ENABLED" : "DISABLED");
#endif
}

void PowerMonitor::enableArmRail(bool on) {
  if (on == armEnabled) return;          // ← only act if state actually changed
  digitalWrite(ARM_EN_PIN, on ? HIGH : LOW);
  armEnabled = on;
#ifdef SERIAL_DEBUG
  Serial.printf("[POWER] ARM rail %s\n", on ? "ENABLED" : "DISABLED");
#endif
}