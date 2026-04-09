#pragma once
#include <Arduino.h>
#include <ESP32Servo.h>
#include "config.h"

class ArmSystem {
public:
  void begin();
  void update(const InputState& currentInput, const InputState& lastInput);

private:
  Servo gripper;
  int gripperAngle;
  unsigned long lastGripperUpdate = 0;

  HardwareSerial lxUart{1}; 
  int lx1_angle = 250;  // Default start to match HOME
  int lx2_angle = 300;  // Default start to match HOME
  unsigned long lastLxUpdate = 0;

  bool isFastMode = false;
  int activeSequenceStep = 0;
  int activeSequenceMode = 0; // 1 = Front, 2 = Back
  unsigned long sequenceTimer = 0;

  // --- N20 Base Stepping Variables ---
  bool isBaseStepping = false;
  unsigned long baseStepTimer = 0;
  const int BASE_STEP_DURATION_MS = 150; // Tune this to make the "step" larger or smaller

  void setBaseMotor(int speed);
  void setGripperAngle(int angle);
  void executePose(int poseID);
  void updateSequence();
  void cancelSequence();
  void handleSerialCalibration();

  void lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len);
  void lx16a_move(uint8_t id, int16_t raw_pos, uint16_t time_ms);

  // === MAPPED POSITIONS ===
  const int HOME_S1 = 250, HOME_S2 = 300;            
  const int PICKUP_S1 = 450, PICKUP_S2 = 1000;        
  const int SAFE_S1 = 900, SAFE_S2 = 400;
  const int FRONT_BIN_S1 = 0, FRONT_BIN_S2 = 525;    
  const int BACK_BIN_S1 = 250, BACK_BIN_S2 = 400;    // <--- UPDATED S2 to 400
};