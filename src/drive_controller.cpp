#include "drive_controller.h"

void DriveController::begin() {
  power.begin();
  motors.begin();
  Serial.println("[CTRL] DriveController + Motors ready");
}

void DriveController::update(const InputState& pkt) {
  // Simple mode-based rail control
  bool wantDrive = (pkt.currentMode == MODE_DRIVE);
  bool wantArm   = (pkt.currentMode == MODE_ARM);

  power.enableDriveRail(wantDrive);
  power.enableArmRail(wantArm);

  motors.update(pkt);

#ifdef SERIAL_DEBUG
  static uint32_t count = 0;
  if (++count % 10 == 0) {
    Serial.printf("[CTRL] Mode:%s  Joy1:%s  Joy2:%s\n",
                  (pkt.currentMode == MODE_DRIVE) ? "DRIVE" : "ARM",
                  dirNames[pkt.joy1], dirNames[pkt.joy2]);
  }
#endif
}