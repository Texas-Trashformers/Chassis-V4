#include "arm_system.h"

void ArmSystem::begin() {
  ESP32PWM::allocateTimer(0);
  gripper.setPeriodHertz(50);
  
  // 1. Wake up the End Effector
  gripper.attach(PWM_EE_PIN, 500, 2500);
  gripperAngle = GRIPPER_MIN;
  gripper.write(gripperAngle);
  
  // --- NEW: Current Sensing Pin ---
  pinMode(CURRENT_SENSOR_PIN, INPUT);

  delay(500);

  pinMode(M_PWMA_PIN, OUTPUT);
  pinMode(M_AIN1_PIN, OUTPUT);
  pinMode(M_AIN2_PIN, OUTPUT);
  setBaseMotor(0); 

  // RAW UART SETUP
  lxUart.begin(115200, SERIAL_8N1, -1, LX16A_SIGNAL_PIN); 

  // 2. Wake up LX-16A Servo 1 (Home S1)
  lx16a_move(1, HOME_S1, 1000);
  lx1_angle = HOME_S1;
  delay(500);

  // 3. Wake up LX-16A Servo 2 (Home S2)
  lx16a_move(2, HOME_S2, 1000);
  lx2_angle = HOME_S2;

  Serial.println("[ARM] Subsystems Initialized (Staggered Startup).");
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
    digitalWrite(M_AIN1_PIN, LOW);
    digitalWrite(M_AIN2_PIN, LOW);
    analogWrite(M_PWMA_PIN, 0);
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
  activeSequenceStep = 0; // Stop the background sequence if user touches joysticks
}

void ArmSystem::executePose(int poseID) {
  int poseSpeed = isFastMode ? 800 : 1500; 
  
  if (poseID == POSE_HOME) {
    lx1_angle = HOME_S1; lx2_angle = HOME_S2;
  } else if (poseID == POSE_PICKUP) {
    lx1_angle = PICKUP_S1; lx2_angle = PICKUP_S2;
  } else if (poseID == POSE_FRONT_BIN) {
    lx1_angle = FRONT_BIN_S1; lx2_angle = FRONT_BIN_S2;
  }

  lx16a_move(1, lx1_angle, poseSpeed);
  lx16a_move(2, lx2_angle, poseSpeed);
}

// Background sequence executor for "Dump Our Back Bin"
void ArmSystem::updateSequence() {
  if (activeSequenceStep == 0) return; // Not running

  // 500ms delay between sequence steps (matches old web controller 'speed' variable)
  if (millis() - sequenceTimer < 500) return;
  sequenceTimer = millis();

  switch(activeSequenceStep) {
    case 1:
      lx1_angle = HOME_S1; lx2_angle = HOME_S2;
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      setGripperAngle(0);
      activeSequenceStep++; break;
    case 2:
      lx1_angle = 125; lx2_angle = 333; // S1=30, S2=80
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      setGripperAngle(0);
      activeSequenceStep++; break;
    case 3:
      setGripperAngle(25);
      activeSequenceStep++; break;
    case 4:
      setGripperAngle(32);
      activeSequenceStep++; break;
    case 5:
      lx1_angle = 42; lx2_angle = 500; // S1=10, S2=120
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      activeSequenceStep++; break;
    case 6:
      lx1_angle = 750; lx2_angle = 500; // S1=180, S2=120
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      activeSequenceStep++; break;
    case 7:
      lx1_angle = 83; lx2_angle = 396; // S1=20, S2=95
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      activeSequenceStep++; break;
    case 8:
      setGripperAngle(0);
      activeSequenceStep++; break;
    case 9:
      lx1_angle = HOME_S1; lx2_angle = HOME_S2;
      lx16a_move(1, lx1_angle, 500); lx16a_move(2, lx2_angle, 500);
      activeSequenceStep = 0; // Sequence Complete
      break;
  }
}

void ArmSystem::update(const InputState& currentInput, const InputState& lastInput) {
  if (currentInput.currentMode != MODE_ARM) {
    setBaseMotor(0); 
    return;
  }

  // --- Step 1: Run active background sequences ---
  updateSequence();

  // --- Speed Mode Toggles (Right D-Pad L/R) ---
  if (currentInput.dpad_R_right && !lastInput.dpad_R_right) isFastMode = true;
  if (currentInput.dpad_R_left && !lastInput.dpad_R_left) isFastMode = false;

  // --- NEW: Smooth LX-16A Steps ---
  int activeLxStep = isFastMode ? 8 : 4; 
  int activeGripperStep = isFastMode ? 8 : GRIPPER_STEP;
  int activeBaseSpeed = isFastMode ? 255 : BASE_SPEED;

  // --- Phase 1: End Effector (Right Joy L/R) ---
  if (millis() - lastGripperUpdate > 50) {
    bool moved = false;
    int proposedAngle = gripperAngle;
    
    int currentDraw = analogRead(CURRENT_SENSOR_PIN); 

    // CLOSING LOGIC WITH STALL PROTECTION
    if (currentInput.joy2 == E || currentInput.joy2 == NE || currentInput.joy2 == SE) {
      if (currentDraw < GRIP_CURRENT_THRESHOLD) { 
        proposedAngle += activeGripperStep; // CLOSE
        moved = true;
      } else {
        // STALL DETECTED: "Back-Off" Method
        proposedAngle -= 2; 
        moved = true; 
      }
    }
    
    // OPENING LOGIC
    if (currentInput.joy2 == W || currentInput.joy2 == NW || currentInput.joy2 == SW) {
      proposedAngle -= activeGripperStep; // OPEN
      moved = true;
    }
    
    if (moved) {
      cancelSequence(); // User takeover
      setGripperAngle(proposedAngle);
    } else if (millis() - lastGripperUpdate > 500 && gripper.attached() && activeSequenceStep == 0) {
      gripper.detach(); // Power saving if idle and no sequence is running
    }
  }

  // --- Phase 2: Base Z-Axis (Left Joy L/R) ---
  if (currentInput.joy1 == E || currentInput.joy1 == NE || currentInput.joy1 == SE) {
    cancelSequence();
    setBaseMotor(activeBaseSpeed);  // CW (Pan Right)
  } else if (currentInput.joy1 == W || currentInput.joy1 == NW || currentInput.joy1 == SW) {
    cancelSequence();
    setBaseMotor(-activeBaseSpeed); // CCW (Pan Left)
  } else {
    setBaseMotor(0);
  }

  // --- Phase 3: LX-16A Joysticks (Left Joy U/D, Right Joy U/D) ---
  // --- NEW: Loop delay increased to 40ms to prevent flooding ---
  if (millis() - lastLxUpdate > 40) { 
    bool lxMoved = false;

    // Servo 1: Lower Arm (Left Joy Up/Down)
    if (currentInput.joy1 == N || currentInput.joy1 == NE || currentInput.joy1 == NW) {
      lx1_angle += activeLxStep; // UP / Lift
      lxMoved = true;
    } else if (currentInput.joy1 == S || currentInput.joy1 == SE || currentInput.joy1 == SW) {
      lx1_angle -= activeLxStep; // DOWN / Lower
      lxMoved = true;
    }

    // Servo 2: Upper Arm (Right Joy Up/Down)
    if (currentInput.joy2 == N || currentInput.joy2 == NE || currentInput.joy2 == NW) {
      lx2_angle += activeLxStep; // UP / Extend
      lxMoved = true;
    } else if (currentInput.joy2 == S || currentInput.joy2 == SE || currentInput.joy2 == SW) {
      lx2_angle -= activeLxStep; // DOWN / Retract
      lxMoved = true;
    }

    if (lxMoved) {
      cancelSequence(); // User takeover

      if (lx1_angle > 1000) lx1_angle = 1000;
      if (lx1_angle < 0) lx1_angle = 0;
      if (lx2_angle > 1000) lx2_angle = 1000;
      if (lx2_angle < 0) lx2_angle = 0;

      // --- NEW: "Chasing the Carrot" Timing (Commanded time is 100ms) ---
      lx16a_move(1, lx1_angle, 100);
      lx16a_move(2, lx2_angle, 100);
      lastLxUpdate = millis();
    }
  }

  // --- Phase 4: Poses (D-Pads) ---
  // Right D-Pad Poses
  if (currentInput.dpad_R_up && !lastInput.dpad_R_up) {
    cancelSequence(); executePose(POSE_HOME);
  }
  if (currentInput.dpad_R_down && !lastInput.dpad_R_down) {
    cancelSequence(); executePose(POSE_PICKUP);
  }

  // Left D-Pad Bin Dumps
  if ((currentInput.dpad_L_up && !lastInput.dpad_L_up) || (currentInput.dpad_L_right && !lastInput.dpad_L_right)) {
    cancelSequence(); executePose(POSE_FRONT_BIN);
  }
  
  // Left D-Pad Down/Left = Execute the complex "Dump Our Back Bin" Sequence
  if ((currentInput.dpad_L_down && !lastInput.dpad_L_down) || (currentInput.dpad_L_left && !lastInput.dpad_L_left)) {
    if (activeSequenceStep == 0) { // Only start if not already running
      activeSequenceStep = 1;
      sequenceTimer = 0; // Trigger step 1 immediately
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