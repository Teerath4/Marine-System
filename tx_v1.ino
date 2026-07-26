#include <Wire.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h> // 🔥 Install TinyGPSPlus library

HardwareSerial Serial2(PA3, PA2); // GPS Module Interface (RX2 = PA3, TX2 = PA2)

#define MPU_ADDR        0x68
#define WATER_PIN       PA0   
#define SOS_BUTTON_PIN  PA6   
#define LED_PIN         PA5   

TinyGPSPlus gps;
unsigned long lastTxTime = 0;
uint16_t seqNum = 0;

// Default Fallbacks (agar satellite lock na ho)
float gpsLat = 13.082680; 
float gpsLon = 80.270718;

void setup() {
  Serial1.begin(9600); // LoRa Channel
  Serial2.begin(9600); // GPS Module Channel
  
  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); 
  Wire.write(0);    
  Wire.endTransmission(true);
  delay(1000);
}

void loop() {
  // 1. Stream & Parse Live Satellites Continuous Data
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      if (gps.location.isValid()) {
        gpsLat = gps.location.lat(); // 🔥 Live Latitude Updated
        gpsLon = gps.location.lng(); // 🔥 Live Longitude Updated
      }
    }
  }

  // 2. Transmit Packet Every 2 Seconds
  if (millis() - lastTxTime >= 2000) {
    lastTxTime = millis();
    float tiltAngle = 0.0;
    
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) == 0) {
      Wire.requestFrom(MPU_ADDR, 6, true);
      if (Wire.available() >= 6) {
        int16_t ax = (Wire.read() << 8) | Wire.read();
        int16_t ay = (Wire.read() << 8) | Wire.read();
        int16_t az = (Wire.read() << 8) | Wire.read();
        tiltAngle = atan2((float)ay, (float)az) * 180.0 / PI;
      }
    }

    int waterRaw = analogRead(WATER_PIN);
    bool localSos = (digitalRead(SOS_BUTTON_PIN) == LOW);

    // Payload strictly structured with Live GPS Coordinates
    String payload = "V1," + String(seqNum++) + "," + String(gpsLat, 6) +
                     "," + String(gpsLon, 6) + "," + String(waterRaw) +
                     "," + String(tiltAngle, 1) + "," + String(localSos ? 1 : 0);
    
    Serial1.print("AT+SEND=0,");
    Serial1.print(payload.length());
    Serial1.print(",");
    Serial1.println(payload);

    digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
  }
}