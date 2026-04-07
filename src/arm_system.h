#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include <HardwareSerial.h>
#include "config.h"

class ArmSystem {
public:
  void begin();
  void update(const InputState& currentInput, const InputState& lastInput);

private:
  // --- Phase 1: End Effector ---
  Servo gripper;
  int gripperAngle = 0; 
  unsigned long lastGripperUpdate = 0;

  // --- Phase 2: N20 Z-Axis Motor ---
  void setBaseMotor(int speed);

  // --- Phase 3: LX-16A Raw Byte Bus ---
  HardwareSerial lxUart{1}; 
  int lx1_angle = 12000; // Centi-degrees (12000 = 120 degrees)
  int lx2_angle = 12000;
  unsigned long lastLxUpdate = 0;
  const int LX_STEP = 300; // 3 degrees per tick

  // Raw serial helpers
  void lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len);
  void lx16a_move(uint8_t id, int16_t centi_deg, uint16_t time_ms);

  // --- Pose Definitions ---
  void executePose(int poseID);
  
  // Angles for each pose in centi-degrees
  const int HOME_S1 = 12000, HOME_S2 = 12000;
  const int PICKUP_S1 = 6000, PICKUP_S2 = 6000;
  const int FRONT_BIN_S1 = 18000, FRONT_BIN_S2 = 18000;
  const int BACK_BIN_S1 = 2000, BACK_BIN_S2 = 2000;
};