#include "motor_driver.h"
#include <HardwareSerial.h>

HardwareSerial lxUart(2);

// ====================== LX16A TRANSMIT-ONLY HELPERS (from your code) ======================
void lx16a_drain() {
  delayMicroseconds(250);
  while (lxUart.available()) lxUart.read();
  delayMicroseconds(80);
}

bool lx16a_send(uint8_t id, uint8_t cmd, uint8_t* params, uint8_t len) {
  if (len > 4) return false;

  uint8_t buflen = 6 + len;
  uint8_t buf[12];

  buf[0] = 0x55; buf[1] = 0x55;
  buf[2] = id;
  buf[3] = 3 + len;
  buf[4] = cmd;
  for (uint8_t i = 0; i < len; i++) buf[5 + i] = params[i];

  uint8_t cksum = 0;
  for (uint8_t i = 2; i < 5 + len; i++) cksum += buf[i];
  buf[5 + len] = ~cksum;

  lx16a_drain();
  lxUart.write(buf, buflen);
  lxUart.flush();

#ifdef SERIAL_DEBUG
  Serial.printf("[LX16A] Sent id=0x%02X cmd=0x%02X\n", id, cmd);
#endif
  return true;
}

void lx16a_go_home() {
  uint8_t p[4] = {0x00, 0x2E, 0x00, 0x00};
  lx16a_send(0xFE, 1, p, 4);
}

void lx16a_move(uint8_t id, int16_t centi_deg) {
  uint16_t a = centi_deg;
  uint8_t p[4] = {(uint8_t)a, (uint8_t)(a >> 8), 0xD0, 0x07};
  lx16a_send(id, 1, p, 4);
}


void TB6612::begin(uint8_t pwmPin, uint8_t in1Pin, uint8_t in2Pin, uint8_t invert) {
  pwm = pwmPin; in1 = in1Pin; in2 = in2Pin; inv = invert;
  pinMode(pwm, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  coast();
}

void TB6612::drive(int16_t speed) {
  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;
  if (inv) speed = -speed;

  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  } else {
    brake();
  }
}

void TB6612::brake() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, HIGH);
  analogWrite(pwm, 0);
}

void TB6612::coast() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(pwm, 0);
}

// ====================== DIAGNOSTIC MOTOR DRIVER ======================
#define INVERT 1
void MotorDriver::begin() {
  leftFront.begin(  L_PWMB, L_BIN1, L_BIN2          );
  leftRear.begin(   L_PWMA, L_AIN1, L_AIN2          );
  rightFront.begin( R_PWMA, R_AIN1, R_AIN2, INVERT  );
  rightRear.begin(  R_PWMB, R_BIN1, R_BIN2, INVERT  );

  // Arm setup
  pinMode(LX16A_SIGNAL_PIN, OUTPUT | OPEN_DRAIN | PULLUP);
  lxUart.begin(115200, SERIAL_8N1, LX16A_SIGNAL_PIN, LX16A_SIGNAL_PIN);
  pinMode(PWM_EE_PIN, OUTPUT);

  // One-time LX-16A 
  lx16a_go_home();
  delay(800);

  Serial.println("[MOTOR] Drive + Arm ready");
}

void MotorDriver::update(const InputState& pkt) {
  if (pkt.currentMode == MODE_DRIVE) {
    handleDriveMode(pkt);
  } else {
    handleArmMode(pkt);
  }
}

void MotorDriver::handleDriveMode(const InputState& pkt){
  int16_t lf = 0, lr = 0, rf = 0, rr = 0;
  int16_t baseSpeed = 150;
  int16_t smallInc  = 80;

  // speed modifiers
  if (pkt.dpad_R_up)    baseSpeed = 220;
  if (pkt.dpad_R_down)  baseSpeed = 55;

  // Strafe from Joy2 left/right
  if (pkt.joy2 == N) { 
    lf = baseSpeed; 
    lr = baseSpeed; 
    rf = baseSpeed; 
    rr = baseSpeed; 
  }
  if (pkt.joy2 == NE) { 
    lf = baseSpeed; 
    lr = 0; 
    rf = 0; 
    rr = baseSpeed; 
  }
  if (pkt.joy2 == NW) { 
    lf = 0; 
    lr = baseSpeed; 
    rf = baseSpeed; 
    rr = 0; 
  }
  if (pkt.joy2 == S) { 
    lf = -baseSpeed; 
    lr = -baseSpeed; 
    rf = -baseSpeed; 
    rr = -baseSpeed;  
  }
  if (pkt.joy2 == SW) { 
    lf = -baseSpeed; 
    lr = 0; 
    rf = 0; 
    rr = -baseSpeed; 
  }
  if (pkt.joy2 == SE) { 
    lf = 0; 
    lr = -baseSpeed; 
    rf = -baseSpeed; 
    rr = 0; 
  }
  if (pkt.joy2 == E) { 
    lf =  baseSpeed; 
    lr = -baseSpeed; 
    rf = -baseSpeed; 
    rr =  baseSpeed; 
  }
  if (pkt.joy2 == W) { 
    lf = -baseSpeed; 
    lr =  baseSpeed; 
    rf =  baseSpeed; 
    rr = -baseSpeed; 
  }

  // Precise movements (small speed)
  if (pkt.dpad_L_up)    { 
    lf = smallInc; 
    lr = smallInc; 
    rf = smallInc; 
    rr = smallInc; 
  }
  if (pkt.dpad_L_down)  { 
    lf = -smallInc; 
    lr = -smallInc; 
    rf = -smallInc;  
    rr = -smallInc; 
  }
  if (pkt.dpad_L_left)  { 
    lf =  smallInc; 
    lr = -smallInc; 
    rf = -smallInc; 
    rr =  smallInc; 
  }
  if (pkt.dpad_L_right) { 
    lf = -smallInc; 
    lr =  smallInc; 
    rf =  smallInc; 
    rr = -smallInc; 
  }

  // Turn in place
  if (pkt.dpad_R_left)  { 
    lf = -baseSpeed; 
    lr = -baseSpeed; 
    rf =  baseSpeed; 
    rr =  baseSpeed; 
  } // Turn left
  if (pkt.dpad_R_right) { 
    lf =  baseSpeed; 
    lr =  baseSpeed; 
    rf = -baseSpeed; 
    rr = -baseSpeed; 
  } // Turn right

  //modifiers from Joy1
  if (pkt.joy1 == NE || pkt.joy1 == E || pkt.joy1 == SE){
    lf += baseSpeed;
    lr += baseSpeed;
    rf -= baseSpeed;
    rr -= baseSpeed;
  }
  if (pkt.joy1 == NW || pkt.joy1 == W || pkt.joy1 == SW){
    lf -= baseSpeed;
    lr -= baseSpeed;
    rf += baseSpeed;
    rr += baseSpeed;
  }

  // Apply to each wheel
  leftFront.drive(lf);
  leftRear.drive(lr);
  rightFront.drive(rf);
  rightRear.drive(rr);
}

void MotorDriver::handleArmMode(const InputState& pkt) {
  // Left D-Pad – Dump commands (LX-16A)
  if (pkt.dpad_L_up)    { lx16a_move(1, 8000); lx16a_move(2, 8000); }     // Dump front bin
  if (pkt.dpad_L_down)  { lx16a_move(1, 16000); lx16a_move(2, 16000); }   // Dump back bin
  if (pkt.dpad_L_left)  { lx16a_move(1, 4000); lx16a_move(2, 4000); }     // Dump to back bin
  if (pkt.dpad_L_right) { lx16a_move(1, 20000); lx16a_move(2, 20000); }   // Dump to front bin

  // Right D-Pad
  if (pkt.dpad_R_up)    { lx16a_go_home(); }                              // Resting / Home
  if (pkt.dpad_R_down)  { lx16a_move(1, 6000); lx16a_move(2, 6000); }    // Pick up in front

  // End effector PWM on GPIO7 – slow ramp with Joy2 E/W
  static int gripperPWM = 0;   // 0 = open, 255 = closed

  if (pkt.joy2 == E) {         // Open gripper slowly
    if (gripperPWM > 0) gripperPWM = max(0, gripperPWM - 6);
  }
  if (pkt.joy2 == W) {         // Close gripper slowly
    if (gripperPWM < 255) gripperPWM = min(255, gripperPWM + 6);
  }

  analogWrite(PWM_EE_PIN, gripperPWM);

#ifdef SERIAL_DEBUG
  static uint32_t count = 0;
  if (++count % 6 == 0) {
    Serial.printf("[ARM] Joy2:%s  Gripper:%d%%  L-Pad:%d%d%d%d\n",
                  dirNames[pkt.joy2], (gripperPWM * 100) / 255,
                  pkt.dpad_L_up, pkt.dpad_L_down, pkt.dpad_L_left, pkt.dpad_L_right);
  }
#endif
}