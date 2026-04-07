#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"
#include "drive_controller.h"
#include "arm_system.h"

// ====================== GLOBAL DEFINITIONS ======================
const char* dirNames[9] = {"CENTER", "N", "NE", "E", "SE", "S", "SW", "W", "NW"};

DriveController driveController;
ArmSystem arm;
volatile unsigned long lastPacketTime = 0;
volatile bool newPacket = false;
InputState latestPacket;
InputState lastPacket;

// ====================== ESP-NOW RECEIVE CALLBACK ======================
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  if (data_len == sizeof(InputState)) {
    memcpy(&latestPacket, data, sizeof(InputState));
    lastPacketTime = millis();
    newPacket = true;
  }
}

void setup() {
  pinMode(WIRELESS_LED_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);

  digitalWrite(WIRELESS_LED_PIN, LOW);
  digitalWrite(FAULT_LED_PIN, LOW);

  Serial.begin(115200);
  Serial.println("\n=== CHASSIS PHASE II – POWER MONITORING ACTIVE ===");

  driveController.begin();           // starts INA219 + rails OFF

  // ESP-NOW setup (clean & direct)
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while(1);
  }
  esp_now_register_recv_cb(onDataRecv);

  Serial.println("[ESP-NOW] Receiver ready on channel 1");
  Serial.println("   My MAC: " + WiFi.macAddress());

  // Startup blink
  digitalWrite(FAULT_LED_PIN, HIGH);
  delay(300);
  digitalWrite(FAULT_LED_PIN, LOW);
  arm.begin();
}

void loop() {
  if (newPacket) {
    newPacket = false;
    digitalWrite(WIRELESS_LED_PIN, HIGH);
    driveController.update(latestPacket);
  }
  
  // Link loss detection
  if (millis() - lastPacketTime > LINK_TIMEOUT_MS) {
    digitalWrite(WIRELESS_LED_PIN, LOW);
    digitalWrite(FAULT_LED_PIN, (millis() % 400 < 200) ? HIGH : LOW);
    
    // --- NEW FAILSAFE: Stop all motors if connection drops ---
    // If we don't do this, the ESP32 will keep trying to drive motors
    // based on stale data, keeping voltage low and preventing reconnection.
    memset(&latestPacket, 0, sizeof(InputState)); 
    // (Assuming 0 equates to 'CENTER' for your joystick enums and unpressed for buttons)
    
  } else {
    digitalWrite(FAULT_LED_PIN, LOW);
  }

  arm.update(latestPacket, lastPacket);
  lastPacket = latestPacket;

  driveController.power.update();   // INA219 readings

  delay(10);
}