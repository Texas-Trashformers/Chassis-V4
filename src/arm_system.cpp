#include "arm_system.h"

void ArmSystem::begin() {
  ESP32PWM::allocateTimer(0);
  gripper.setPeriodHertz(50);
  
  gripper.attach(PWM_EE_PIN, 500, 2500);
  gripperAngle = GRIPPER_MIN;
  gripper.write(gripperAngle);
  
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  pinMode(ARM_EN_PIN, OUTPUT);
  digitalWrite(ARM_EN_PIN, HIGH); 

  delay(500);

  pinMode(M_PWMA_PIN, OUTPUT);
  pinMode(M_AIN1_PIN, OUTPUT);
  pinMode(M_AIN2_PIN, OUTPUT);
  setBaseMotor(0); 

  lxUart.begin(115200, SERIAL_8N1, -1, LX16A_SIGNAL_PIN); 

  lx16a_move(1, HOME_S1, 2000);
  lx1_angle = HOME_S1;
  delay(500);

  lx16a_move(2, HOME_S2, 2000);
  lx2_angle = HOME_S2;
}

void ArmSystem::setBaseMotor(int speed) {
  static int currentSpeed = -999;
  if (speed == currentSpeed) return; 
  currentSpeed = speed;

  if (speed > 0) {
    digitalWrite(M_AIN1_PIN, HIGH);
    digitalWrite(M_AIN2_PIN, LOW);
    analogWrite(M_PWMA_PIN, speed);
  } else if (speed < 0) {
    digitalWrite(M_AIN1_PIN, LOW);
    digitalWrite(M_AIN2_PIN, HIGH);
    analogWrite(M_PWMA_PIN, abs(speed));
  } else {
    // Active Braking
    digitalWrite(M_AIN1_PIN, HIGH);
    digitalWrite(M_AIN2_PIN, HIGH);
    analogWrite(M_PWMA_PIN, 255);
  }
}

void ArmSystem::setGripperAngle(int angle) {
  if (angle > GRIPPER_MAX) angle = GRIPPER_MAX;
  if (angle < GRIPPER_MIN) angle = GRIPPER_MIN;
  gripperAngle = angle;
  
  if (!gripper.attached()) {
    gripper.attach(PWM_EE_PIN, 500, 2500); 
  }
  gripper.write(gripperAngle);
  lastGripperUpdate = millis();
}

void ArmSystem::cancelSequence() {
  activeSequenceStep = 0; 
  activeSequenceMode = 0;
}

void ArmSystem::executePose(int poseID) {
  int poseSpeed = 2000; 
  
  if (poseID == POSE_HOME) {
    lx1_angle = HOME_S1; lx2_angle = HOME_S2; setGripperAngle(0);
  } else if (poseID == POSE_PICKUP) {
    lx1_angle = PICKUP_S1; lx2_angle = PICKUP_S2; setGripperAngle(0);
  }

  lx16a_move(1, lx1_angle, poseSpeed);
  lx16a_move(2, lx2_angle, poseSpeed);
}

void ArmSystem::updateSequence() {
  if (activeSequenceStep == 0) return; 

  // --- CHANGED: 75% Speed. Old: 800ms. New: 1066ms ---
  if (millis() - sequenceTimer < 1066) return;
  sequenceTimer = millis();

  switch(activeSequenceStep) {
    case 1: // STEP 1: Move to SAFE
      lx1_angle = SAFE_S1; lx2_angle = SAFE_S2;
      lx16a_move(1, lx1_angle, 1066); lx16a_move(2, lx2_angle, 1066);
      activeSequenceStep++; break;
      
    case 2: // STEP 2: Move to Target Bin 
      if (activeSequenceMode == 1) { // Front Bin
        lx1_angle = FRONT_BIN_S1; lx2_angle = FRONT_BIN_S2;
      } else { // Back Bin (Now uses updated BACK_BIN_S2 of 400)
        lx1_angle = BACK_BIN_S1; lx2_angle = BACK_BIN_S2;
      }
      lx16a_move(1, lx1_angle, 1066); lx16a_move(2, lx2_angle, 1066);
      activeSequenceStep++; break;

    case 3: // STEP 3: Hold the grip! (No changes to gripperAngle)
      // Pauses for 1066ms at the bin position while maintaining grip torque
      activeSequenceStep++; break;

    case 4: // STEP 4: Move back to SAFE to clear bin edge
      lx1_angle = SAFE_S1; lx2_angle = SAFE_S2;
      lx16a_move(1, lx1_angle, 1066); lx16a_move(2, lx2_angle, 1066);
      activeSequenceStep++; break;

    case 5: // STEP 5: Move to PICKUP
      lx1_angle = PICKUP_S1; lx2_angle = PICKUP_S2;
      lx16a_move(1, lx1_angle, 1066); lx16a_move(2, lx2_angle, 1066);
      activeSequenceStep = 0; // End sequence
      break;
  }
}

void ArmSystem::update(const InputState& currentInput, const InputState& lastInput) {
  handleSerialCalibration();

  if (currentInput.currentMode != MODE_ARM) {
    setBaseMotor(0); 
    isBaseStepping = false;
    return;
  }

  updateSequence();

  int activeLxStep = 6; 
  int activeGripperStep = GRIPPER_STEP;
  int activeBaseSpeed = BASE_SPEED;

  // --- N20 BASE MOTOR (Right D-Pad L/R) ---
  if (currentInput.dpad_R_right && !lastInput.dpad_R_right && !isBaseStepping) {
    cancelSequence();
    isBaseStepping = true;
    baseStepTimer = millis();
    setBaseMotor(activeBaseSpeed);  
  } 
  else if (currentInput.dpad_R_left && !lastInput.dpad_R_left && !isBaseStepping) {
    cancelSequence();
    isBaseStepping = true;
    baseStepTimer = millis();
    setBaseMotor(-activeBaseSpeed); 
  }

  if (isBaseStepping) {
    if (millis() - baseStepTimer >= BASE_STEP_DURATION_MS) {
      isBaseStepping = false;
      setBaseMotor(0); 
    }
  }

  // --- GRIPPER CONTROL (Right Joy E/W) ---
  if (millis() - lastGripperUpdate > 50) {
    bool moved = false;
    int proposedAngle = gripperAngle;
    int currentDraw = analogRead(CURRENT_SENSOR_PIN); 

    if (currentInput.joy2 == E || currentInput.joy2 == NE || currentInput.joy2 == SE) {
      if (currentDraw < GRIP_CURRENT_THRESHOLD) { 
        proposedAngle += activeGripperStep; 
        moved = true;
      } else {
        proposedAngle -= 2; 
        moved = true; 
      }
    }
    
    if (currentInput.joy2 == W || currentInput.joy2 == NW || currentInput.joy2 == SW) {
      proposedAngle -= activeGripperStep; 
      moved = true;
    }
    
    if (moved) {
      cancelSequence(); 
      setGripperAngle(proposedAngle);
    } else if (millis() - lastGripperUpdate > 500 && gripper.attached() && activeSequenceStep == 0) {
      gripper.detach(); 
    }
  }

  // --- LX-16A ARM CONTROL ---
  if (millis() - lastLxUpdate > 40) { 
    bool lxMoved = false;

    if (currentInput.joy1 == N || currentInput.joy1 == NE || currentInput.joy1 == NW) {
      lx1_angle += activeLxStep; lxMoved = true;
    } else if (currentInput.joy1 == S || currentInput.joy1 == SE || currentInput.joy1 == SW) {
      lx1_angle -= activeLxStep; lxMoved = true;
    }

    if (currentInput.joy2 == N || currentInput.joy2 == NE || currentInput.joy2 == NW) {
      lx2_angle += activeLxStep; lxMoved = true;
    } else if (currentInput.joy2 == S || currentInput.joy2 == SE || currentInput.joy2 == SW) {
      lx2_angle -= activeLxStep; lxMoved = true;
    }

    if (lxMoved) {
      cancelSequence(); 

      if (lx1_angle > 1000) lx1_angle = 1000;
      if (lx1_angle < 0) lx1_angle = 0;
      if (lx2_angle > 1000) lx2_angle = 1000;
      if (lx2_angle < 0) lx2_angle = 0;

      lx16a_move(1, lx1_angle, 100);
      lx16a_move(2, lx2_angle, 100);
      lastLxUpdate = millis();
    }
  }

  // --- SINGLE PRESS BUTTONS ---
  if (currentInput.dpad_R_up && !lastInput.dpad_R_up) {
    cancelSequence(); executePose(POSE_HOME);
  }
  if (currentInput.dpad_R_down && !lastInput.dpad_R_down) {
    cancelSequence(); executePose(POSE_PICKUP);
  }

  if (currentInput.dpad_L_up && !lastInput.dpad_L_up) {
    if (activeSequenceStep == 0) { 
      activeSequenceMode = 1; 
      activeSequenceStep = 1;
      sequenceTimer = 0; 
    }
  }
  
  if (currentInput.dpad_L_down && !lastInput.dpad_L_down) {
    if (activeSequenceStep == 0) { 
      activeSequenceMode = 2; 
      activeSequenceStep = 1;
      sequenceTimer = 0; 
    }
  }
}

// ==========================================
// LX-16A RAW BYTE HELPERS
// ==========================================
void ArmSystem::lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len) {
  uint8_t buf[12];
  buf[0] = 0x55; 
  buf[1] = 0x55;
  buf[2] = id;
  buf[3] = 3 + len;
  buf[4] = cmd;
  
  for (uint8_t i = 0; i < len; i++) {
    buf[5 + i] = params[i];
  }

  uint8_t cksum = 0;
  for (uint8_t i = 2; i < 5 + len; i++) {
    cksum += buf[i];
  }
  buf[5 + len] = ~cksum;

  lxUart.write(buf, 6 + len);
}

void ArmSystem::lx16a_move(uint8_t id, int16_t raw_pos, uint16_t time_ms) {
  uint8_t p[4] = {
    (uint8_t)(raw_pos & 0xFF), 
    (uint8_t)(raw_pos >> 8), 
    (uint8_t)(time_ms & 0xFF), 
    (uint8_t)(time_ms >> 8)
  };
  lx16a_send(id, 1, p, 4); 
}

// ==========================================
// SERIAL CALIBRATION TOOL
// ==========================================
void ArmSystem::handleSerialCalibration() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) return;

    if (input.equalsIgnoreCase("P")) {
      Serial.println("\n--- CURRENT ARM ANGLES ---");
      Serial.printf("LX1 (Lower Arm) : %d\n", lx1_angle);
      Serial.printf("LX2 (Upper Arm) : %d\n", lx2_angle);
      Serial.printf("Gripper         : %d\n", gripperAngle);
      Serial.println("--------------------------\n");
      return;
    }

    int spaceIndex = input.indexOf(' ');
    if (spaceIndex == -1 || spaceIndex == input.length() - 1) {
      Serial.println("[ERROR] Incomplete command. Make sure you include a space (e.g., '1 500').");
      return;
    }

    char cmd = input.charAt(0);
    int val = input.substring(spaceIndex + 1).toInt();

    cancelSequence(); 

    if (cmd == '1') {
      if (val > 1000) val = 1000;
      if (val < 0) val = 0;
      lx1_angle = val;
      lx16a_move(1, lx1_angle, 1000); 
      Serial.printf("[CALIBRATE] LX1 moved to %d\n", lx1_angle);
    } 
    else if (cmd == '2') {
      if (val > 1000) val = 1000;
      if (val < 0) val = 0;
      lx2_angle = val;
      lx16a_move(2, lx2_angle, 1000); 
      Serial.printf("[CALIBRATE] LX2 moved to %d\n", lx2_angle);
    } 
    else if (cmd == 'G' || cmd == 'g') {
      setGripperAngle(val);
      Serial.printf("[CALIBRATE] Gripper moved to %d\n", gripperAngle);
    } 
    else {
      Serial.println("[ERROR] Invalid. Format: '1 [val]', '2 [val]', 'G [val]', or 'P' to print.");
    }
  }
}