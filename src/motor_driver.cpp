#include "motor_driver.h"
#include <HardwareSerial.h>



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

  Serial.println("[MOTOR] Drive ready");
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
  // All arm logic is now handled by arm_system.cpp!
  // When in Arm Mode, the motor driver just needs to safely stop the wheels.
  
  leftFront.brake();
  leftRear.brake();
  rightFront.brake();
  rightRear.brake();
}