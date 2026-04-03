#pragma once
#include <Arduino.h>

// ======================= RECEIVER CONFIG =======================

// ====================== INPUT STATE STRUCT (MUST MATCH CONTROLLER EXACTLY) ======================
enum joystickpos {C, N, NE, E, SE, S, SW, W, NW};

const char* dirNames[9] = {"CENTER", "N", "NE", "E", "SE", "S", "SW", "W", "NW"};

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

// ====================== TIMING ======================
const unsigned long LINK_TIMEOUT_MS = 500;   // if no packet for this long → fault LED