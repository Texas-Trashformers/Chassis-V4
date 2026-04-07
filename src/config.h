#pragma once
#include <Arduino.h>

// ======================= RECEIVER CONFIG =======================

// ====================== INPUT STATE STRUCT (MUST MATCH CONTROLLER EXACTLY) ======================
enum joystickpos {C, N, NE, E, SE, S, SW, W, NW};

extern const char* dirNames[9];

enum ControlMode { 
  MODE_DRIVE = 0,
  MODE_ARM 
};

typedef struct __attribute__((packed)) {
  ControlMode currentMode;
  bool dpad_L_up;
  bool dpad_L_down;
  bool dpad_L_left;
  bool dpad_L_right;
  bool dpad_R_up;
  bool dpad_R_down;
  bool dpad_R_left;
  bool dpad_R_right;
  bool btn_R1;      // L1 is ignored as requested
  joystickpos joy1;
  joystickpos joy2;
} InputState;

// ====================== DEBUG ======================
#define SERIAL_DEBUG

// ====================== LEDs ======================
#define WIRELESS_LED_PIN   12
#define FAULT_LED_PIN      13

// ====================== HIGH-SIDE SWITCHES ======================
#define DRIVE_EN_PIN       8
#define ARM_EN_PIN         4

// ====================== OTHER FUTURE PINS (placeholders - we'll fill later) ======================
#define LX16A_SIGNAL_PIN   5    // LX-16A bus servo signal
#define SCL_PIN            1
#define SDA_PIN            2

// ====================== PHASE III: MOTOR CONTROL PINS ======================
// Arm TB6612FNG (M_* group)
#define M_PWMA            15
#define M_PWMB            16
#define M_AIN1            39
#define M_AIN2            40
#define M_BIN1            41
#define M_BIN2            42

// Drive Right TB6612FNG (R_* group)
#define R_PWMA            14
#define R_AIN1            17
#define R_AIN2            18
#define R_PWMB            21
#define R_BIN1            35
#define R_BIN2            36

// Drive Left TB6612FNG (L_* group)
#define L_PWMA            37
#define L_AIN1            38
#define L_AIN2            45
#define L_PWMB            46
#define L_BIN1            47
#define L_BIN2            48

// LX-16A Bus Servo Signal
#define LX16A_SIGNAL_PIN   5

// End Effector Standard PWM Servo
#define PWM_EE_PIN         7

// Limit Switches
#define LIM_SW_L_PIN       9   
#define LIM_SW_R_PIN      10   

// ====================== ARM SYSTEM ======================
#define GRIPPER_MIN     0
#define GRIPPER_MAX     180
#define GRIPPER_STEP    5
#define BASE_SPEED      150

#define POSE_HOME       0
#define POSE_PICKUP     1
#define POSE_FRONT_BIN  2
#define POSE_BACK_BIN   3

// Pin aliases used by arm_system
#define M_PWMA_PIN      M_PWMA
#define M_AIN1_PIN      M_AIN1
#define M_AIN2_PIN      M_AIN2

// ====================== TIMING ======================
const unsigned long LINK_TIMEOUT_MS = 500;   // if no packet for this long → fault LED