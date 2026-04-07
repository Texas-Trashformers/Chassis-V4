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
  void setGripperAngle(int angle); // Helper to handle power-saving attach/detach

  // --- Phase 2: N20 Z-Axis Motor ---
  void setBaseMotor(int speed);

  // --- Phase 3: LX-16A Raw Byte Bus ---
  HardwareSerial lxUart{1}; 
  int lx1_angle = 83;  // Home S1 (20 deg)
  int lx2_angle = 416; // Home S2 (100 deg)
  unsigned long lastLxUpdate = 0;

  // Raw serial helpers
  void lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len);
  void lx16a_move(uint8_t id, int16_t raw_pos, uint16_t time_ms);

  // --- Pose & Sequence Definitions ---
  void executePose(int poseID);
  
  // Non-blocking sequence variables
  int activeSequenceStep = 0;
  unsigned long sequenceTimer = 0;
  void updateSequence();
  void cancelSequence();

  // Translated Angles (Degrees * 1000 / 240)
  const int HOME_S1 = 83, HOME_S2 = 416;             // 20 deg, 100 deg
  const int PICKUP_S1 = 416, PICKUP_S2 = 958;        // 100 deg, 230 deg
  const int FRONT_BIN_S1 = 0, FRONT_BIN_S2 = 708;    // 0 deg, 170 deg
  const int BACK_BIN_S1 = 333, BACK_BIN_S2 = 416;    // 80 deg, 100 deg
};