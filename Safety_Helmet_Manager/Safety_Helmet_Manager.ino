#include <LoRa.h>
#include <SPI.h>

// LoRa Pins (same as slave)
#define LORA_SCK 14
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 18
#define LORA_RST 22
#define LORA_DI0 26

void setup() {
  Serial.begin(115200);
  
  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(433E6)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  Serial.println("LoRa master init succeeded.");
  
  // Set LoRa parameters (must match slaves)
  LoRa.setSyncWord(0xF3);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
}

void loop() {
  // Check for incoming messages
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String received = "";
    while (LoRa.available()) {
      received += (char)LoRa.read();
    }
    
    Serial.println("Received: " + received);
    
    // Process message
    if (received.indexOf("HUMIDITY:") != -1) {
      // Log humidity data
      Serial.println("Humidity update: " + received);
    } 
    else if (received.indexOf("_ALERT") != -1) {
      // Broadcast alert to all helmets
      Serial.println("Danger detected! Broadcasting alert...");
      broadcastAlert(received);
    }
  }
  
  delay(100);
}

void broadcastAlert(String originalAlert) {
  // Forward alert to all helmets with ALERT: prefix
  String broadcastMessage = "ALERT:" + originalAlert;
  LoRa.beginPacket();
  LoRa.print(broadcastMessage);
  LoRa.endPacket();
  Serial.println("Broadcasted: " + broadcastMessage);
}
