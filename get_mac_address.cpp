#include <Arduino.h>
#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("ESP32 MAC Address Information:");
  Serial.println("==============================");
  
  // Get WiFi MAC address
  WiFi.mode(WIFI_STA);
  String macAddress = WiFi.macAddress();
  
  Serial.print("WiFi MAC Address: ");
  Serial.println(macAddress);
  
  // Alternative format (without colons)
  macAddress.replace(":", "");
  Serial.print("MAC (no colons): ");
  Serial.println(macAddress);
  
  // Get as byte array
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC as bytes: ");
  for (int i = 0; i < 6; i++) {
    Serial.print("0x");
    if (mac[i] < 16) Serial.print("0");
    Serial.print(mac[i], HEX);
    if (i < 5) Serial.print(", ");
  }
  Serial.println();
  
  Serial.println("\nMAC address retrieved successfully!");
}

void loop() {
  // Do nothing
  delay(1000);
}