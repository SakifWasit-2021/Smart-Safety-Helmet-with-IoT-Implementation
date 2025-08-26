#include <Wire.h>
#include <BH1750.h>
#include <DHT.h>
#include <WiFi.h>
#include <SPI.h>

// ========== PIN DEFINITIONS ==========
#define BUTTON_PIN 12
#define TRIG_PIN 13
#define ECHO_PIN 27
#define BUZZER_PIN 15
#define BLUE_LED_PIN 16
#define MQ2_PIN 34
#define DHT_PIN 17
#define BATTERY_PIN 35
#define RED_BAT_LED_PIN 4
#define GREEN_BAT_LED_PIN 5
#define HELMET_LEDS_PIN 21
#define PINK_LED_PIN 25

// LoRa Pins
#define LORA_SCK 14
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 18
#define LORA_RST 22
#define LORA_DI0 26

// ========== LED CONTROL ==========
#define LED_BLUE 0
#define LED_PINK 1
#define LED_HELMET 2
#define NUM_LEDS 3

struct LEDBlinker {
  int pin;
  unsigned long interval;
  unsigned long duration;
  unsigned long startTime;
  unsigned long lastToggle;
  bool state;
  bool active;
};

LEDBlinker ledBlinkers[NUM_LEDS] = {
  {BLUE_LED_PIN, 0, 0, 0, 0, LOW, false},  // LED_BLUE
  {PINK_LED_PIN, 0, 0, 0, 0, LOW, false},  // LED_PINK
  {HELMET_LEDS_PIN, 0, 0, 0, 0, LOW, false} // LED_HELMET
};

// ========== CONSTANTS ==========
#define TEMP_THRESHOLD_HIGH 40.0
#define TEMP_THRESHOLD_LOW 10.0
#define HUMIDITY_THRESHOLD_HIGH 70.0
#define HUMIDITY_THRESHOLD_LOW 20.0
#define DISTANCE_THRESHOLD 3.0
#define LUX_THRESHOLD 50
#define BATTERY_LOW 7.0
#define BATTERY_HIGH 8.4
#define LORA_FREQUENCY 433E6
#define ALERT_DURATION 10000

// ========== GLOBALS ==========
BH1750 lightMeter;
DHT dht(DHT_PIN, DHT11);

String helmetID = "HELMET_01";
unsigned long lastAlertTime = 0;
bool hazardDetected = false;
unsigned long buzzerStartTime = 0;
unsigned long buzzerDuration = 0;
bool isBuzzerOn = false;
unsigned long lastBuzzerToggle = 0;
unsigned long lastSensorRead = 0;
const unsigned long SENSOR_READ_INTERVAL = 1000;

void setup() {
  Serial.begin(115200);
  
  // Initialize pins
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_BAT_LED_PIN, OUTPUT);
  pinMode(GREEN_BAT_LED_PIN, OUTPUT);
  
  // Initialize LED pins through startLEDBlink()
  for (int i = 0; i < NUM_LEDS; i++) {
    pinMode(ledBlinkers[i].pin, OUTPUT);
    digitalWrite(ledBlinkers[i].pin, LOW);
  }
  
  // Initialize sensors
  Wire.begin();
  lightMeter.begin();
  dht.begin();
  
  // Initialize LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DI0);
  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  LoRa.setSyncWord(0xF3);
  LoRa.setSpreadingFactor(12);
  LoRa.setSignalBandwidth(125E3);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Update all active LED blink patterns
  updateLEDBlinks(currentMillis);
  
  // Handle buzzer timing (non-blocking)
  updateBuzzer(currentMillis);
  
  // Sensor readings (throttled)
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentMillis;
    
    // 1. Hazardous gas detection
    checkGas();
    
    // 2. Temperature and humidity monitoring
    checkTemperatureAndHumidity();
    
    // 3. Light level monitoring
    checkLightLevel();
    
    // 4. Battery monitoring
    checkBattery();
  }
  
  // 5. Button-triggered distance measurement
  if (digitalRead(BUTTON_PIN) == LOW) {
    checkDistance();
  }
  
  // 6. Handle danger alerts
  handleDangerAlerts(currentMillis);
  
  // 7. Check for incoming LoRa messages
  onReceive(LoRa.parsePacket());
}

// ========== LED CONTROL FUNCTIONS ==========
void startLEDBlink(int ledIndex, unsigned long totalDuration, unsigned long blinkInterval) {
  if (ledIndex >= NUM_LEDS) return;
  
  ledBlinkers[ledIndex].interval = blinkInterval;
  ledBlinkers[ledIndex].duration = totalDuration;
  ledBlinkers[ledIndex].startTime = millis();
  ledBlinkers[ledIndex].lastToggle = millis();
  ledBlinkers[ledIndex].state = LOW;
  ledBlinkers[ledIndex].active = true;
  digitalWrite(ledBlinkers[ledIndex].pin, LOW);
}

void stopLEDBlink(int ledIndex) {
  if (ledIndex >= NUM_LEDS) return;
  
  ledBlinkers[ledIndex].active = false;
  digitalWrite(ledBlinkers[ledIndex].pin, LOW);
}

void updateLEDBlinks(unsigned long currentMillis) {
  for (int i = 0; i < NUM_LEDS; i++) {
    if (!ledBlinkers[i].active) continue;

    // Check if duration has expired
    if (ledBlinkers[i].duration > 0 && 
        (currentMillis - ledBlinkers[i].startTime) >= ledBlinkers[i].duration) {
      stopLEDBlink(i);
      continue;
    }

    // Time to toggle?
    if (currentMillis - ledBlinkers[i].lastToggle >= ledBlinkers[i].interval) {
      ledBlinkers[i].state = !ledBlinkers[i].state;
      digitalWrite(ledBlinkers[i].pin, ledBlinkers[i].state);
      ledBlinkers[i].lastToggle = currentMillis;
    }
  }
}

// ========== BUZZER CONTROL ==========
void updateBuzzer(unsigned long currentMillis) {
  if (isBuzzerOn && buzzerDuration > 0 && 
      currentMillis - buzzerStartTime >= buzzerDuration) {
    stopBuzzer();
  }
}

void startBuzzer(unsigned long duration) {
  isBuzzerOn = true;
  buzzerStartTime=millis();
  buzzerDuration= duration;
  digitalWrite(BUZZER_PIN, HIGH);
  
}

void stopBuzzer() {
  isBuzzerOn = false;
  digitalWrite(BUZZER_PIN, LOW);
}

// ========== SENSOR FUNCTIONS ==========

// ========== HCSR-04 ALERT FUNCTIONS ==========
void checkDistance() {
  float distance = getAverageDistance(3);
  
  if (distance > DISTANCE_THRESHOLD) {
    // Non-blocking beep pattern
    startBuzzer(200);
    startLEDBlink(LED_BLUE, 10000, 500); // Fast blink for 10s
    triggerDanger("OBSTACLE:" + String(distance, 1));
  } else {
    startLEDBlink(LED_BLUE, 2000, 500); // Normal blink for 2s
  }
}

float getAverageDistance(int samples) {
  float sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += getDistance();
    delay(25); // Small delay between readings
  }
  return sum / samples;
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

// ========== MQ-2 ALERT FUNCTIONS ==========
void checkGas() {
  int gasValue = analogRead(MQ2_PIN);
  if (gasValue > 1500) {
    startBuzzer(0); // Continuous until cleared
    startLEDBlink(LED_BLUE, 0, 200); // Rapid blink
    triggerDanger("GAS_ALERT:" + String(gasValue));
  } else {
    stopBuzzer();
    stopLEDBlink(LED_BLUE);
  }
}

// ========== DHT11 ALERT FUNCTIONS ==========
void checkTemperatureAndHumidity() {
  float temp = dht.readTemperature();
  float humidity = dht.readHumidity();
  
  if (isnan(temp) || isnan(humidity)) {
    Serial.println("DHT read failed!");
    return;
  }

  // Temperature alerts
  if (temp > TEMP_THRESHOLD_HIGH || temp < TEMP_THRESHOLD_LOW) {
    startBuzzer(1000); // Long beep
    startLEDBlink(LED_BLUE, 0, 1000); // Slow blink
    triggerDanger("TEMP_ALERT:" + String(temp, 1));
  } else {
    stopBuzzer();
  }

  // Humidity alerts
  if (humidity > HUMIDITY_THRESHOLD_HIGH) {
    startLEDBlink(LED_PINK, 0, 500); // Medium blink
  } else if (humidity < HUMIDITY_THRESHOLD_LOW) {
    stopLEDBlink(LED_PINK);
    digitalWrite(PINK_LED_PIN, HIGH); // Solid on
  } else {
    digitalWrite(PINK_LED_PIN, LOW); // Off
  }

  // Regular humidity updates
  sendToMaster("HUMIDITY:" + String(humidity, 1));
}

// ========== HEADLIGHT ON/OFF FUNCTION ==========
void checkLightLevel() {
  float lux = lightMeter.readLightLevel();
  digitalWrite(HELMET_LEDS_PIN, lux < LUX_THRESHOLD ? HIGH : LOW);
}

// ========== CHECKING BATTERY CONDITION ==========
void checkBattery() {
  int rawValue = analogRead(BATTERY_PIN);
  float voltage = rawValue * (3.3 / 4095.0) * 2.23;
  
  digitalWrite(RED_BAT_LED_PIN, voltage < BATTERY_LOW ? HIGH : LOW);
  digitalWrite(GREEN_BAT_LED_PIN, 
    (voltage >= BATTERY_LOW && voltage <= BATTERY_HIGH) ? HIGH : LOW);
}

// ========== ALERT FUNCTIONS ==========
void triggerDanger(String message) {
  hazardDetected = true;
  lastAlertTime = millis();
  sendLoRaMessage(helmetID + ":" + message);
}

void handleDangerAlerts(unsigned long currentMillis) {
  if (hazardDetected && (currentMillis - lastAlertTime >= ALERT_DURATION)) {
    hazardDetected = false;
    // Any cleanup when alert period ends
  }
}

// ========== LoRa COMMUNICATION ==========
void sendLoRaMessage(String message) {
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();
  Serial.println("Sent: " + message);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;
  
  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }
  
  if (received.startsWith("ALERT:")) {
    hazardDetected = true;
    lastAlertTime = millis();
    startLEDBlink(LED_BLUE, ALERT_DURATION, 500);
  }
}