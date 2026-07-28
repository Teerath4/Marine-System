/*
 * MINIMAL TWO-WAY LORA LINK TEST — no AUX pin used
 * Flash this identical file to BOTH boards — just change NODE_NAME below
 * before flashing each one.
 *
 * LoRa module wiring:
 *   TX  ----- PA10 (STM32 RX1)
 *   RX  ----- PA9  (STM32 TX1)
 *   M0  ----- GND
 *   M1  ----- GND
 *   VCC ----- 3.3V
 *   GND ----- GND
 *
 * Debug TTL adapter wiring (MOVED off PA9/PA10 since LoRa is there now):
 *   Adapter RX ----- PA2 (STM32 TX2)
 *   Adapter TX ----- PA3 (STM32 RX2)
 *   Adapter GND ---- STM32 GND
 */

#define NODE_NAME "B"   // <-- change to "B" on the second board before flashing

#include "Arduino.h"
#include "LoRa_E22.h"

// No AUX, no M0/M1 pins passed — M0/M1 are hardwired to GND externally,
// so the library doesn't need to control them in software.
LoRa_E22 e22ttl(&Serial1);

unsigned long lastSend = 0;
uint16_t counter = 0;

void setup() {
  Serial2.begin(9600);   // debug output — now on PA2/PA3
  delay(500);

  e22ttl.begin();

  Serial2.println();
  Serial2.print(F("=== NODE "));
  Serial2.print(NODE_NAME);
  Serial2.println(F(" — LINK TEST ==="));
}

void loop() {
  if (millis() - lastSend >= 2000) {
    lastSend = millis();
    String msg = String(NODE_NAME) + ",seq=" + String(counter++);
    ResponseStatus rs = e22ttl.sendMessage(msg);
    Serial2.print(F("SENT: "));
    Serial2.print(msg);
    Serial2.print(F("  ["));
    Serial2.print(rs.getResponseDescription());
    Serial2.println(F("]"));
  }

  if (e22ttl.available() > 1) {
    ResponseContainer rc = e22ttl.receiveMessage();
    Serial2.print(F("RECV: "));
    Serial2.println(rc.data);
  }
}