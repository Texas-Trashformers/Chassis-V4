#include "drive_system.h"

void DriveSystem::begin() {
  // Set all driver control pins as outputs
  const int outputPins[] = {
    L_PWMA_PIN, L_AIN1_PIN, L_AIN2_PIN, L_PWMB_PIN, L_BIN1_PIN, L_BIN2_PIN,
    R_PWMA_PIN, R_AIN1_PIN, R_AIN2_PIN, R_PWMB_PIN, R_BIN1_PIN, R_BIN2_PIN
  };
  
  for (int pin : outputPins) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  
  Serial.println("[DRIVE] Mecanum Subsystem Initialized");
}

void DriveSystem::getJoystickVector(joystickpos pos, int &x, int &y) {
  x = 0; y = 0;
  switch(pos) {
    case N:  x =  0; y =  1; break;
    case S:  x =  0; y = -1; break;
    case E:  x =  1; y =  0; break;
    case W:  x = -1; y =  0; break;
    case NE: x =  1; y =  1; break;
    case NW: x = -1; y =  1; break;
    case SE: x =  1; y = -1; break;
    case SW: x = -1; y = -1; break;
    case C:  x =  0; y =  0; break;
  }
}

void DriveSystem::update(const InputState& input) {
  // 1. Handle Speed Modes
  if (input.dpad_R_right) fastMode = true;
  if (input.dpad_R_left)  fastMode = false;
  currentSpeedLimit = fastMode ? 255 : 120; // 255 = max speed, 120 = precision speed

  // 2. Get Translation (Left Joystick) and Rotation (Right Joystick X-axis)
  int transX, transY, rotX, rotY;
  getJoystickVector(input.joy1, transX, transY);
  getJoystickVector(input.joy2, rotX, rotY); 
  
  // Micro-adjustments (D-Pad Left) override translation if pressed
  if (input.dpad_L_up)    { transX =  0; transY =  1; }
  if (input.dpad_L_down)  { transX =  0; transY = -1; }
  if (input.dpad_L_right) { transX =  1; transY =  0; }
  if (input.dpad_L_left)  { transX = -1; transY =  0; }

  int rotation = rotX; // Left/Right on right stick dictates rotation

  // 3. Standard Mecanum Kinematics Formula
  // Assuming Motor A = Front, Motor B = Back
  int frontLeft  = transY + transX + rotation;
  int backLeft   = transY - transX + rotation;
  int frontRight = transY - transX - rotation;
  int backRight  = transY + transX - rotation;

  // 4. Normalize speeds so we don't exceed our PWM limit
  int maxVal = max({abs(frontLeft), abs(backLeft), abs(frontRight), abs(backRight), 1});
  
  frontLeft  = (frontLeft  * currentSpeedLimit) / maxVal;
  backLeft   = (backLeft   * currentSpeedLimit) / maxVal;
  frontRight = (frontRight * currentSpeedLimit) / maxVal;
  backRight  = (backRight  * currentSpeedLimit) / maxVal;

  // If no input is given, stop immediately
  if (transX == 0 && transY == 0 && rotation == 0) {
    frontLeft = backLeft = frontRight = backRight = 0;
  }

  // 5. Output to TB6612 Hardware
  setMotor(L_PWMA_PIN, L_AIN1_PIN, L_AIN2_PIN, frontLeft);  // Front Left
  setMotor(L_PWMB_PIN, L_BIN1_PIN, L_BIN2_PIN, backLeft);   // Back Left
  setMotor(R_PWMA_PIN, R_AIN1_PIN, R_AIN2_PIN, frontRight); // Front Right
  setMotor(R_PWMB_PIN, R_BIN1_PIN, R_BIN2_PIN, backRight);  // Back Right
}

void DriveSystem::setMotor(int pwmPin, int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, speed);
  } else if (speed < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
    analogWrite(pwmPin, abs(speed));
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    analogWrite(pwmPin, 0); // Stop
  }
}