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
  // --- Speed Mode Tracking ---
  bool isFastMode = false; 

  // --- Phase 1: End Effector ---
  Servo gripper;
  int gripperAngle = 0; 
  unsigned long lastGripperUpdate = 0;

  // --- Phase 2: N20 Z-Axis Motor ---
  void setBaseMotor(int speed);

  // --- Phase 3: LX-16A Raw Byte Bus ---
  HardwareSerial lxUart{1}; 
  int lx1_angle = 500; // 500 = Center (120 degrees)
  int lx2_angle = 500;
  unsigned long lastLxUpdate = 0;

  // Raw serial helpers
  void lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len);
  void lx16a_move(uint8_t id, int16_t raw_pos, uint16_t time_ms);

  // --- Pose Definitions ---
  void executePose(int poseID);
  
  // Angles mapped to the 0-1000 range (0 = 0 deg, 1000 = 240 deg)
  const int HOME_S1 = 500, HOME_S2 = 500;             // 120 deg
  const int PICKUP_S1 = 250, PICKUP_S2 = 250;         // 60 deg
  const int FRONT_BIN_S1 = 750, FRONT_BIN_S2 = 750;   // 180 deg
  const int BACK_BIN_S1 = 83, BACK_BIN_S2 = 83;       // 20 deg
};