#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "config.h"

InputState rxPacket = {MODE_DRIVE};
unsigned long lastPacketTime = 0;

// ====================== ESP-NOW RECEIVE CALLBACK ======================
void onDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
  Serial.printf("[RX] Raw packet received! length=%d (expected %d)\n", data_len, sizeof(InputState));

  if (data_len == sizeof(InputState)) {
    InputState pkt;
    memcpy(&pkt, data, sizeof(InputState));
    lastPacketTime = millis();
    digitalWrite(WIRELESS_LED_PIN, HIGH);

    uint16_t mask = (pkt.dpad_L_up << 0) | (pkt.dpad_L_down << 1) | (pkt.dpad_L_left << 2) |
                    (pkt.dpad_L_right << 3) | (pkt.dpad_R_up << 4) | (pkt.dpad_R_down << 5) |
                    (pkt.dpad_R_left << 6) | (pkt.dpad_R_right << 7) | (pkt.btn_R1 << 8);

    Serial.printf("[RX] SUCCESS → Mode:%s  Joy1:%s  Joy2:%s  Buttons:0x%04X\n",
                  (pkt.currentMode == MODE_DRIVE) ? "DRIVE" : "ARM",
                  dirNames[pkt.joy1], dirNames[pkt.joy2], mask);
  } else {
    Serial.println("[RX] WRONG SIZE - packet ignored");
  }
}

// ====================== SETUP ======================
void setup() {
  pinMode(WIRELESS_LED_PIN, OUTPUT);
  pinMode(FAULT_LED_PIN, OUTPUT);
  pinMode(DRIVE_EN_PIN, OUTPUT);
  pinMode(ARM_EN_PIN, OUTPUT);

  digitalWrite(WIRELESS_LED_PIN, LOW);
  digitalWrite(FAULT_LED_PIN, LOW);
  digitalWrite(DRIVE_EN_PIN, LOW);
  digitalWrite(ARM_EN_PIN, LOW);

  Serial.begin(115200);
  Serial.println("\n=== CHASSIS RECEIVER (DIAGNOSTIC BUILD) ===");

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);   // try channel 1 first

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    while(1);
  }

  esp_now_register_recv_cb(onDataRecv);
  Serial.println("[OK] Callback registered successfully");

  Serial.printf("Expected packet size = %d bytes\n", sizeof(InputState));
  Serial.println("   My MAC: " + WiFi.macAddress());
  Serial.println("Waiting for broadcast packets from controller...");

  // Startup blink
  digitalWrite(FAULT_LED_PIN, HIGH);
  delay(300);
  digitalWrite(FAULT_LED_PIN, LOW);
}

// ====================== LOOP ======================
void loop() {
  // Heartbeat so we know the loop is running
  static unsigned long lastHeartbeat = 0;
  if (millis() - lastHeartbeat > 2000) {
    Serial.println("[HEARTBEAT] Receiver loop is alive");
    lastHeartbeat = millis();
  }

  // Link loss detection
  if (millis() - lastPacketTime > LINK_TIMEOUT_MS) {
    digitalWrite(WIRELESS_LED_PIN, LOW);
    digitalWrite(FAULT_LED_PIN, (millis() % 400 < 200) ? HIGH : LOW);
  } else {
    digitalWrite(FAULT_LED_PIN, LOW);
  }

  delay(10);
}