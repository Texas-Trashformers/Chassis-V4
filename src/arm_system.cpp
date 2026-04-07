#include "arm_system.h"

void ArmSystem::begin() {
  ESP32PWM::allocateTimer(0);
  gripper.setPeriodHertz(50);
  
  // 1. Wake up the End Effector
  gripper.attach(PWM_EE_PIN, 500, 2500);
  gripperAngle = GRIPPER_MIN;
  gripper.write(gripperAngle);
  
  delay(500); // <-- Let power stabilize

  pinMode(M_PWMA_PIN, OUTPUT);
  pinMode(M_AIN1_PIN, OUTPUT);
  pinMode(M_AIN2_PIN, OUTPUT);
  setBaseMotor(0); 

  // RAW UART SETUP
  lxUart.begin(115200, SERIAL_8N1, -1, LX16A_SIGNAL_PIN); 

  // 2. Wake up LX-16A Servo 1
  lx16a_move(1, HOME_S1, 1000);
  lx1_angle = HOME_S1;
  
  delay(500); // <-- Let power stabilize

  // 3. Wake up LX-16A Servo 2
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

void ArmSystem::executePose(int poseID) {
  int poseSpeed = isFastMode ? 800 : 1500; // Reach pose faster in fast mode
  
  if (poseID == POSE_HOME) {
    lx1_angle = HOME_S1; lx2_angle = HOME_S2;
  } else if (poseID == POSE_PICKUP) {
    lx1_angle = PICKUP_S1; lx2_angle = PICKUP_S2;
  } else if (poseID == POSE_FRONT_BIN) {
    lx1_angle = FRONT_BIN_S1; lx2_angle = FRONT_BIN_S2;
  } else if (poseID == POSE_BACK_BIN) {
    lx1_angle = BACK_BIN_S1; lx2_angle = BACK_BIN_S2;
  }

  lx16a_move(1, lx1_angle, poseSpeed);
  lx16a_move(2, lx2_angle, poseSpeed);
}

void ArmSystem::update(const InputState& currentInput, const InputState& lastInput) {
  if (currentInput.currentMode != MODE_ARM) {
    setBaseMotor(0); 
    return;
  }

  // --- Speed Mode Toggles (Right D-Pad L/R) ---
  if (currentInput.dpad_R_right && !lastInput.dpad_R_right) isFastMode = true;
  if (currentInput.dpad_R_left && !lastInput.dpad_R_left) isFastMode = false;

  // Dynamic Speeds based on mode
  int activeLxStep = isFastMode ? 5 : 2;
  int activeGripperStep = isFastMode ? 8 : GRIPPER_STEP;
  int activeBaseSpeed = isFastMode ? 255 : BASE_SPEED;

  // --- Phase 1: End Effector (Right Joy L/R) ---
  if (millis() - lastGripperUpdate > 50) {
    bool moved = false;
    
    // Right Joystick Left/Right maps to Open/Close
    // Note: Swapping + and - here will reverse the open/close direction if it's backwards on your specific servo
    if ((currentInput.joy2 == E || currentInput.joy2 == NE || currentInput.joy2 == SE) && gripperAngle < GRIPPER_MAX) {
      gripperAngle += activeGripperStep; // CLOSE
      moved = true;
    }
    if ((currentInput.joy2 == W || currentInput.joy2 == NW || currentInput.joy2 == SW) && gripperAngle > GRIPPER_MIN) {
      gripperAngle -= activeGripperStep; // OPEN
      moved = true;
    }
    
    if (gripperAngle > GRIPPER_MAX) gripperAngle = GRIPPER_MAX;
    if (gripperAngle < GRIPPER_MIN) gripperAngle = GRIPPER_MIN;
    
    if (moved) {
      if (!gripper.attached()) {
        gripper.attach(PWM_EE_PIN, 500, 2500); 
      }
      gripper.write(gripperAngle);
      lastGripperUpdate = millis();
    } else if (millis() - lastGripperUpdate > 500 && gripper.attached()) {
      gripper.detach(); 
    }
  }

  // --- Phase 2: Base Z-Axis (Left Joy L/R) ---
  if (currentInput.joy1 == E || currentInput.joy1 == NE || currentInput.joy1 == SE) {
    setBaseMotor(activeBaseSpeed);  // CW (Pan Right)
  } else if (currentInput.joy1 == W || currentInput.joy1 == NW || currentInput.joy1 == SW) {
    setBaseMotor(-activeBaseSpeed); // CCW (Pan Left)
  } else {
    setBaseMotor(0);
  }

  // --- Phase 3: LX-16A Joysticks (Left Joy U/D, Right Joy U/D) ---
  if (millis() - lastLxUpdate > 30) {
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
      if (lx1_angle > 1000) lx1_angle = 1000;
      if (lx1_angle < 0) lx1_angle = 0;
      if (lx2_angle > 1000) lx2_angle = 1000;
      if (lx2_angle < 0) lx2_angle = 0;

      lx16a_move(1, lx1_angle, 30);
      lx16a_move(2, lx2_angle, 30);
      lastLxUpdate = millis();
    }
  }

  // --- Phase 4: Poses (D-Pads) ---
  // Right D-Pad Poses
  if (currentInput.dpad_R_up && !lastInput.dpad_R_up) {
    executePose(POSE_HOME);
  }
  if (currentInput.dpad_R_down && !lastInput.dpad_R_down) {
    executePose(POSE_PICKUP);
  }

  // Left D-Pad Bin Dumps (Left/Down = Back, Right/Up = Front)
  if ((currentInput.dpad_L_up && !lastInput.dpad_L_up) || (currentInput.dpad_L_right && !lastInput.dpad_L_right)) {
    executePose(POSE_FRONT_BIN);
  }
  if ((currentInput.dpad_L_down && !lastInput.dpad_L_down) || (currentInput.dpad_L_left && !lastInput.dpad_L_left)) {
    executePose(POSE_BACK_BIN);
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