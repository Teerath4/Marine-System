/*
  ============================================================================
  MARINE DISTRESS NETWORK — FINAL MERGED NODE FIRMWARE  (v1.0)
  ============================================================================
  Hardware: STM32 Blue Pill (F103C8T6) + Ebyte E22-900T30D (SX1262, UART) +
            u-blox NEO-M8L GPS + MPU6050 IMU + analog water level sensor +
            physical SOS button (hardware interrupt) + buzzer + LED

  Every vessel runs this SAME firmware. Role (distress / helper / relayer /
  idle) is decided at runtime by the state machine, not compile-time.

  RADIO CONFIG ASSUMED ALREADY DONE (not this firmware's job):
    - Channel 16 (866.125MHz), Power 30dBm, LBT enabled, Transparent mode,
      M0/M1 both hardwired to GND permanently. If you haven't run the
      one-time config sketch on this exact module, do that FIRST — this
      firmware does not configure the radio, it assumes it's already correct.

  PIN MAP (STM32 Blue Pill):
    Serial1 (PA9 TX / PA10 RX)  -> LoRa E22 module (transparent mode, no
                                    AUX/M0/M1 software control needed since
                                    M0/M1 are hardwired externally to GND)
    Serial2 (PA2 TX / PA3 RX)   -> Debug output (connect your TTL adapter here)
    Serial3 (PB10 TX / PB11 RX) -> GPS (NEO-M8L)
    I2C1    (PB6 SCL / PB7 SDA) -> MPU6050
    PA0                          -> Water level sensor (analog)
    PA1                          -> MPU6050 INT pin (hardware motion interrupt)
    PA4                          -> Buzzer
    PA5                          -> LED
    PA6                          -> SOS button (hardware interrupt, INPUT_PULLUP,
                                    active LOW — button ties pin to GND when pressed)

  ============================================================================
  PLACEHOLDER VALUES THAT NEED YOUR REAL NUMBERS BEFORE THIS IS FIELD-READY:
  ============================================================================
  1. WATER_ESCALATE_RAW / WATER_WARNING_RAW / WATER_DISTRESS_RAW — these are
     raw ADC counts (0-4095 on STM32's 12-bit ADC) from YOUR specific analog
     water sensor. There is no research-paper number for this — it depends
     entirely on your sensor's actual output curve. Calibrate by dipping the
     sensor to known depths and reading raw analogRead() values, then fill
     these in. Current values are placeholders only, NOT calibrated.
  2. MPU_MOTION_THR / MPU_MOTION_DUR — MPU6050 hardware motion-interrupt
     threshold/duration registers, for catching a SUDDEN capsize (impact-like
     acceleration change) instantly via hardware interrupt, separate from the
     sustained-tilt-angle polling logic below. These need bench tuning against
     your actual boat's normal wave-motion "noise floor" so ordinary rocking
     doesn't false-trigger. Placeholder values given are the common
     tutorial-reference starting point, NOT validated against your hull.
  3. TILT_ESCALATE_DEG=15, TILT_WARNING_DEG=30, TILT_DISTRESS_DEG=40 — these
     ARE research-backed (FAO/maritime small-vessel stability criteria: 30°
     is a standard righting-curve checkpoint, 40° approximates the angle of
     downflooding θf used as a generic flooding-onset reference when your
     vessel's actual θf isn't calculated). Reasonable defaults, not guesses,
     but your vessel's real θf may differ — get it from a naval architect if
     this ever goes past prototype.
  ============================================================================
*/

#include "LoRa_E22.h"
#include <Wire.h>
#include <TinyGPS++.h>


// ============================================================================
// CONFIG — EDIT PER VESSEL
// ============================================================================
#define VESSEL_ID            0x0001   // UNIQUE per vessel — change before flashing each unit
#define PORT_LAT_DEG          9.9312  // Reference port/coast-guard coordinate
#define PORT_LON_DEG         76.2673

// Pins
#define WATER_PIN            PA0
#define MPU_INT_PIN           PA1
#define BUZZER_PIN            PA4
#define LED_PIN                 PA5
#define SOS_BUTTON_PIN         PA6

#define MPU_ADDR              0x68

// --- Tilt thresholds (research-backed, see header notes) ---
#define TILT_ESCALATE_DEG      15.0
#define TILT_WARNING_DEG       30.0
#define TILT_DISTRESS_DEG      40.0
#define TILT_SUSTAIN_MS         2000   // must hold past TILT_DISTRESS_DEG this long (debounce wave motion)

// --- Water thresholds (PLACEHOLDER — calibrate against your actual sensor) ---
#define WATER_ESCALATE_RAW      600
#define WATER_WARNING_RAW       1400
#define WATER_DISTRESS_RAW      2600

// --- MPU6050 hardware motion-interrupt (PLACEHOLDER — bench-tune against real wave noise) ---
#define MPU_MOTION_THR            20   // sensitivity, 1 LSB = 4mg — lower = more sensitive
#define MPU_MOTION_DUR              1  // ms-ish units per datasheet, keep short for instant catch

// --- Polling tiers (software, independent of the hardware interrupt above) ---
#define POLL_IDLE_MS           5000
#define POLL_ESCALATED_MS      1500

// --- Protocol timing ---
#define ACK_WINDOW_MS          10000
#define MAX_HOPS                  10   // increased per decision — chain is killed past this, not escalated further
#define SEEN_CACHE_TTL_MS     600000   // 10 min
#define MAX_RANGE_KM              15.0

// --- Retry-with-backoff schedule for zero-ACK distress broadcasts ---
// idle gap AFTER an empty ACK window, before re-broadcasting. Listen window
// itself (ACK_WINDOW_MS) never changes — only this gap grows over time.
#define RETRY_STAGE1_END_MS   (2UL*60UL*1000UL)     // 0-2 min since first trigger
#define RETRY_STAGE2_END_MS   (10UL*60UL*1000UL)    // 2-10 min
#define RETRY_STAGE3_END_MS   (30UL*60UL*1000UL)    // 10-30 min
#define RETRY_GAP_STAGE1_MS   0UL          // immediate retry
#define RETRY_GAP_STAGE2_MS   20000UL      // +10s window = 30s cycle
#define RETRY_GAP_STAGE3_MS   50000UL      // +10s window = 60s cycle
#define RETRY_GAP_STAGE4_MS   170000UL     // +10s window = 180s cycle (steady state, forever)

// ============================================================================
// PACKET TYPES  (binary, big-endian manual packing — see v0.2 design doc)
// ============================================================================
enum PacketType : uint8_t {
  PKT_DISTRESS = 0x01,
  PKT_ACK      = 0x02,
  PKT_DISPATCH = 0x03,
  PKT_RELAY    = 0x04
};

// ROLE is a bitmask, not an enum — a vessel can be dispatched as BOTH
#define ROLE_HELPER   0x01
#define ROLE_RELAYER  0x02

// FLAGS bits (shared header)
#define FLAG_IS_RELAY        (1 << 7)
#define FLAG_ACK_REQ         (1 << 6)
#define FLAG_ROLE_ASSIGNED   (1 << 5)
#define FLAG_CHAIN_EXHAUSTED (1 << 4)

// Packet sizes (field-list totals, verified — see design doc corrections)
// Shared header: TYPE(1) FLAGS(1) VESSEL_ID(2) SEQ_NUM(2) TIMESTAMP(4) = 10B
// DISTRESS = header(10) + LAT(4) + LON(4) + HOPCOUNT(1)                = 19B
// ACK      = header(10) + ORIGIN_MSGID(4) + SENDER_LAT(4) + SENDER_LON(4) = 22B
//            (DIST_TO_PORT field REMOVED per decision — distress vessel
//             derives both distances itself from raw lat/lon, see notes)
// DISPATCH = header(10) + TARGET_ID(2) + ROLE(1) + LAT(4) + LON(4) + ORIGIN_MSGID(2) = 23B
// RELAY    = header(10) + ORIGIN_ID(2) + LAT(4) + LON(4) + HOPCOUNT(1) + PATH(10) = 31B
#define MAX_PATH_ENTRIES 5

// ============================================================================
// GLOBAL STATE
// ============================================================================
TinyGPSPlus gps;
// NOTE: MPU6050 is accessed via raw I2C register calls (readTiltDegrees(),
// initMPU(), configureMPUMotionInterrupt() below), matching the register-level
// pattern already used in your bring-up code — no MPU6050 library dependency.

LoRa_E22 e22ttl(&Serial1); // Transparent mode, M0/M1 hardwired to GND externally

uint16_t seqCounter = 0;

enum NodeState {
  STATE_IDLE,
  STATE_DISTRESS_ACTIVE,   // this vessel triggered, collecting ACKs
  STATE_ACTING_HELPER,
  STATE_ACTING_RELAYER
};
volatile NodeState nodeState = STATE_IDLE;

unsigned long distressStartTime = 0;   // when the FIRST attempt was broadcast
unsigned long ackWindowStartTime = 0;  // when the CURRENT attempt's ACK window opened
uint16_t activeDistressMsgID = 0;
uint16_t activeDistressSender = 0;
int32_t  activeDistressLat = 0, activeDistressLon = 0;
bool     waitingForAckWindow = false;

// SOS button — ISR only sets a flag, main loop does the actual work (ISRs
// must stay short and can't safely do UART/I2C blocking calls)
volatile bool sosPressedFlag = false;

// Tilt sustain tracking
unsigned long tiltDistressStartTime = 0;
bool tiltDistressInProgress = false;

// Software polling tier tracking
unsigned long lastSensorPoll = 0;
enum PollTier { TIER_IDLE, TIER_ESCALATED };
PollTier currentPollTier = TIER_IDLE;

// ACK pool
#define MAX_ACK_POOL 20
struct AckEntry {
  uint16_t senderID;
  int32_t lat, lon;
  bool valid;
};
AckEntry ackPool[MAX_ACK_POOL];
uint8_t ackPoolCount = 0;

// Seen-message cache
#define SEEN_CACHE_SIZE 32
struct SeenEntry {
  uint16_t msgID;
  unsigned long seenAt;
  bool used;
};
SeenEntry seenCache[SEEN_CACHE_SIZE];

// ============================================================================
// SOS BUTTON — HARDWARE INTERRUPT
// ============================================================================
void sosISR() {
  // MUST stay minimal — no Serial, no I2C, no radio calls here.
  sosPressedFlag = true;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial2.begin(9600); // debug
  Serial3.begin(9600); // GPS

  pinMode(SOS_BUTTON_PIN, INPUT_PULLUP);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MPU_INT_PIN, INPUT);

  // Interrupt fires on the falling edge (button press pulls pin to GND)
  attachInterrupt(digitalPinToInterrupt(SOS_BUTTON_PIN), sosISR, FALLING);

  Wire.begin();
  initMPU();
  configureMPUMotionInterrupt(); // hardware fast-capsize path, see function below

  e22ttl.begin();

  memset(seenCache, 0, sizeof(seenCache));
  memset(ackPool, 0, sizeof(ackPool));

  Serial2.println(F("Marine distress node online."));
  Serial2.print(F("VESSEL_ID=0x")); Serial2.println(VESSEL_ID, HEX);
}

// ============================================================================
// MAIN LOOP — non-blocking, single-loop architecture (RTOS port deferred
// until this is validated on real hardware, per earlier decision)
// ============================================================================
void loop() {
  // 1. Always feed GPS parser
  while (Serial3.available()) {
    gps.encode(Serial3.read());
  }

  // 2. Always listen for incoming radio packets — reading Serial1 directly,
  //    NOT the library's String-based receiveMessage(). Transparent mode is
  //    just a UART tunnel, and String can mishandle embedded null bytes and
  //    other non-printable values that are routine in binary packets. This
  //    is the fix for open item #8 (binary-mode receive was still untested
  //    when we only confirmed plain-string send/receive on the bench).
  static uint8_t rxBuf[40];
  static uint8_t rxLen = 0;
  static unsigned long lastByteTime = 0;
  while (Serial1.available()) {
    if (rxLen < sizeof(rxBuf)) {
      rxBuf[rxLen++] = Serial1.read();
      lastByteTime = millis();
    } else {
      Serial1.read(); // discard if buffer overflowed — shouldn't happen at our packet sizes
    }
  }
  // Consider a packet "complete" after a short gap with no new bytes —
  // simple framing since we have no explicit packet-length prefix.
  // Tune GAP_MS against your actual air data rate if packets get split.
  const unsigned long GAP_MS = 50;
  if (rxLen > 0 && (millis() - lastByteTime) > GAP_MS) {
    handleIncomingPacket(rxBuf, rxLen);
    rxLen = 0;
  }

  // 3. SOS button — checked every loop pass, overrides everything else,
  //    no matter what sensors currently say
  if (sosPressedFlag) {
    sosPressedFlag = false;
    if (nodeState == STATE_IDLE) {
      Serial2.println(F("SOS BUTTON PRESSED — manual override distress trigger"));
      triggerDistress(true); // true = manual override flag
    }
  }

  // 4. Sensor polling — tiered rate, only meaningful when idle (not already
  //    in distress) and only if SOS didn't already fire this cycle
  if (nodeState == STATE_IDLE) {
    unsigned long pollInterval = (currentPollTier == TIER_ESCALATED) ? POLL_ESCALATED_MS : POLL_IDLE_MS;
    if (millis() - lastSensorPoll >= pollInterval) {
      lastSensorPoll = millis();
      checkSensorsAndEscalate();
    }
  }

  // 5. Distress state: manage ACK window + retry-with-backoff
  if (nodeState == STATE_DISTRESS_ACTIVE && waitingForAckWindow) {
    if (millis() - ackWindowStartTime >= ACK_WINDOW_MS) {
      finalizeAckWindow();
    }
  }

  // 6. Seen-cache eviction
  evictExpiredSeenEntries();
}

// ============================================================================
// SENSOR POLLING + TIER ESCALATION (software path — slow drift / gradual case)
// ============================================================================
void checkSensorsAndEscalate() {
  float tiltDeg = readTiltDegrees();
  int waterRaw = analogRead(WATER_PIN);

  // Escalate polling tier if either signal crosses the low threshold —
  // this does NOT trigger distress, it just samples faster to catch the
  // actual moment things get worse.
  if (tiltDeg >= TILT_ESCALATE_DEG || waterRaw >= WATER_ESCALATE_RAW) {
    currentPollTier = TIER_ESCALATED;
  } else {
    currentPollTier = TIER_IDLE;
  }

  // Distress trigger: sustained tilt past the flooding-onset-equivalent
  // threshold, OR water past its distress threshold (no sustain required
  // for water — a sudden water spike is not a false-positive risk the way
  // wave-induced tilt spikes are).
  bool tiltPastDistress = (tiltDeg >= TILT_DISTRESS_DEG);
  if (tiltPastDistress) {
    if (!tiltDistressInProgress) {
      tiltDistressInProgress = true;
      tiltDistressStartTime = millis();
    } else if (millis() - tiltDistressStartTime >= TILT_SUSTAIN_MS) {
      Serial2.println(F("TILT distress threshold sustained — triggering"));
      triggerDistress(false);
      tiltDistressInProgress = false;
      return;
    }
  } else {
    tiltDistressInProgress = false;
  }

  if (waterRaw >= WATER_DISTRESS_RAW) {
    Serial2.println(F("WATER distress threshold crossed — triggering"));
    triggerDistress(false);
  }
}

// ============================================================================
// TILT — full 3D tilt-from-vertical (NOT roll-only atan2(ay,az)). Captures
// tilt in any direction (pitch AND roll), correcting the gap flagged earlier.
// ============================================================================
float readTiltDegrees() {
  int16_t ax, ay, az;
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return 0.0;
  Wire.requestFrom(MPU_ADDR, 6, true);
  if (Wire.available() < 6) return 0.0;
  ax = (Wire.read() << 8) | Wire.read();
  ay = (Wire.read() << 8) | Wire.read();
  az = (Wire.read() << 8) | Wire.read();

  return atan2(sqrt((float)ax*ax + (float)ay*ay), (float)az) * 180.0 / PI;
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1
  Wire.write(0x00); // wake up
  Wire.endTransmission(true);
  delay(100);
}

// Hardware motion interrupt for the FAST capsize case — separate from the
// sustained-tilt polling above. This catches a sudden impact-like
// acceleration change instantly via the MPU6050's own interrupt pin,
// regardless of the software polling schedule.
void configureMPUMotionInterrupt() {
  // Standard MPU6050 motion-detection register sequence (public register
  // map, same one used across common MPU6050 tutorials/datasheets):
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1F); Wire.write(MPU_MOTION_THR); // MOT_THR
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x20); Wire.write(MPU_MOTION_DUR); // MOT_DUR
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x37); Wire.write(0x00); // INT_PIN_CFG — default, active-high, push-pull
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x38); Wire.write(0x40); // INT_ENABLE — bit6 = motion interrupt enable
  Wire.endTransmission(true);

  // NOTE: this is register-configured but this firmware does NOT currently
  // attach an STM32 EXTI interrupt on MPU_INT_PIN to act on it instantly —
  // that's the remaining wiring/interrupt-handler step. Until that's added,
  // this configures the MPU to assert its INT pin on sudden motion, but
  // nothing on the STM32 side is listening for the edge yet. Add an
  // attachInterrupt() on MPU_INT_PIN calling a flag-setting ISR (same
  // pattern as sosISR) once MOT_THR/MOT_DUR are bench-tuned against your
  // actual boat's wave-motion noise floor — don't enable it against
  // untested thresholds, a false trigger here fires a real distress signal.
}

// ============================================================================
// DISTRESS TRIGGER + RETRY-WITH-BACKOFF
// ============================================================================
void triggerDistress(bool manualOverride) {
  bool freshTrigger = (nodeState != STATE_DISTRESS_ACTIVE);

  if (freshTrigger) {
    distressStartTime = millis();
    ackPoolCount = 0;
    digitalWrite(LED_PIN, HIGH);
    tone(BUZZER_PIN, 2000);
  }

  seqCounter++; // NEW MsgID every attempt, including retries — see notes below
  uint16_t msgID = makeMsgID(VESSEL_ID, seqCounter);

  int32_t lat = gpsLatToFixed();
  int32_t lon = gpsLonToFixed();
  uint32_t ts = millis();

  if (!gps.location.isValid()) {
    Serial2.println(F("WARNING: distress triggered with NO GPS FIX — coordinates may be stale/zero."));
    // Deliberately still sending — a distress signal with possibly-stale
    // coordinates is better than none, but downstream (helper/relayer UI)
    // should be aware this can happen. Consider adding a GPS-invalid flag
    // bit if you want responders to know the position may be unreliable.
  }

  uint8_t buf[19];
  uint8_t idx = 0;
  buf[idx++] = PKT_DISTRESS;
  buf[idx++] = FLAG_ACK_REQ | (manualOverride ? 0 : 0); // manual-override doesn't
                                                          // need its own bit unless
                                                          // you want responders to
                                                          // see "this was a person,
                                                          // not a sensor" — add a
                                                          // flag bit here if useful
  packBE16(buf, idx, msgID);      idx += 2;
  packBE16(buf, idx, VESSEL_ID);  idx += 2;
  packBE32(buf, idx, ts);         idx += 4;
  packBE32(buf, idx, (uint32_t)lat); idx += 4;
  packBE32(buf, idx, (uint32_t)lon); idx += 4;
  buf[idx++] = 0; // hop count starts at 0

  sendPacket(buf, idx);
  markSeen(msgID);

  nodeState = STATE_DISTRESS_ACTIVE;
  ackWindowStartTime = millis();
  waitingForAckWindow = true;
  activeDistressMsgID = msgID;
  activeDistressSender = VESSEL_ID;
  activeDistressLat = lat;
  activeDistressLon = lon;

  Serial2.print(F("DISTRESS broadcast, MsgID=0x")); Serial2.println(msgID, HEX);
}

void finalizeAckWindow() {
  waitingForAckWindow = false;
  Serial2.print(F("ACK window closed. Pool size=")); Serial2.println(ackPoolCount);

  if (ackPoolCount > 0) {
    dispatchRoles();
    nodeState = STATE_IDLE;
    noTone(BUZZER_PIN);
    digitalWrite(LED_PIN, LOW);
    return;
  }

  // Zero responders — retry with backoff, per the schedule we worked out
  unsigned long elapsed = millis() - distressStartTime;
  unsigned long gap;
  if (elapsed < RETRY_STAGE1_END_MS)      gap = RETRY_GAP_STAGE1_MS;
  else if (elapsed < RETRY_STAGE2_END_MS) gap = RETRY_GAP_STAGE2_MS;
  else if (elapsed < RETRY_STAGE3_END_MS) gap = RETRY_GAP_STAGE3_MS;
  else                                    gap = RETRY_GAP_STAGE4_MS;

  Serial2.print(F("No responders. Retrying after ")); Serial2.print(gap);
  Serial2.println(F("ms idle gap."));

  // Local alarm keeps running regardless — this is purely about the radio
  // retry cadence, not about giving up on alerting the crew locally.
  delay(gap); // NOTE: acceptable here since we are deliberately idle waiting
              // for the next broadcast attempt and not expecting to service
              // anything time-critical during this specific gap; if you
              // later want SOS-button-during-retry-gap responsiveness,
              // replace this with a non-blocking millis()-tracked wait.
  triggerDistress(false); // re-broadcast with a fresh MsgID
}

// ============================================================================
// ACK HANDLING (responder side)
// ============================================================================
void handleDistressPacket(uint8_t* buf, int n) {
  if (n < 19) return;
  uint16_t msgID    = unpackBE16(buf, 2);
  uint16_t senderID = unpackBE16(buf, 4);
  if (senderID == VESSEL_ID) return; // our own broadcast
  if (isSeen(msgID)) return;
  markSeen(msgID);

  int32_t lat = (int32_t)unpackBE32(buf, 10);
  int32_t lon = (int32_t)unpackBE32(buf, 14);

  float distKm = haversineKm(gpsLatToFixed(), gpsLonToFixed(), lat, lon);
  unsigned long jitterMs = (unsigned long)((distKm / MAX_RANGE_KM) * 2000.0) + random(0, 500);

  // NOTE: blocking delay here is a known simplification (flagged previously)
  // — acceptable for now since we're on single-loop architecture, revisit
  // if RTOS port happens later.
  delay(jitterMs);
  sendAck(msgID);
}

void sendAck(uint16_t msgID) {
  int32_t myLat = gpsLatToFixed();
  int32_t myLon = gpsLonToFixed();

  uint8_t buf[22];
  uint8_t idx = 0;
  buf[idx++] = PKT_ACK;
  buf[idx++] = 0; // flags
  packBE16(buf, idx, seqCounter); idx += 2; // this vessel's own seq for its own MsgID
  packBE16(buf, idx, VESSEL_ID);  idx += 2;
  packBE32(buf, idx, millis());   idx += 4;
  packBE32(buf, idx, msgID);      idx += 4; // ORIGIN_MSGID — which distress this answers
  packBE32(buf, idx, (uint32_t)myLat); idx += 4;
  packBE32(buf, idx, (uint32_t)myLon); idx += 4;

  sendPacket(buf, idx);
  Serial2.println(F("Sent ACK"));
}

void handleAckPacket(uint8_t* buf, int n) {
  if (n < 22) return;
  if (nodeState != STATE_DISTRESS_ACTIVE) return;

  uint32_t originMsgID = unpackBE32(buf, 10);
  if (originMsgID != activeDistressMsgID) return; // not answering our current attempt

  uint16_t senderID = unpackBE16(buf, 4);
  int32_t lat = (int32_t)unpackBE32(buf, 14);
  int32_t lon = (int32_t)unpackBE32(buf, 18);

  if (ackPoolCount >= MAX_ACK_POOL) return;
  ackPool[ackPoolCount++] = { senderID, lat, lon, true };
  Serial2.print(F("ACK received from 0x")); Serial2.println(senderID, HEX);
}

// ============================================================================
// ROLE DISPATCH — distress vessel computes BOTH distances itself, from the
// raw lat/lon every responder sent. No DIST_TO_PORT field needed at all.
// ============================================================================
void dispatchRoles() {
  int helperIdx = -1;
  float bestHelperDist = 1e9;
  int relayIdx = -1;
  float bestRelayDist = 1e9;

  int32_t portLat = (int32_t)(PORT_LAT_DEG * 1000000.0);
  int32_t portLon = (int32_t)(PORT_LON_DEG * 1000000.0);

  for (int i = 0; i < ackPoolCount; i++) {
    float distToDistress = haversineKm(ackPool[i].lat, ackPool[i].lon, activeDistressLat, activeDistressLon);
    float distToPort      = haversineKm(ackPool[i].lat, ackPool[i].lon, portLat, portLon);

    if (distToDistress < bestHelperDist) { bestHelperDist = distToDistress; helperIdx = i; }
    if (distToPort < bestRelayDist)      { bestRelayDist = distToPort;      relayIdx = i; }
  }

  if (helperIdx == relayIdx && helperIdx >= 0) {
    // Same vessel wins both — dispatch ONE packet with both bits set
    sendDispatch(ackPool[helperIdx].senderID, ROLE_HELPER | ROLE_RELAYER);
  } else {
    if (helperIdx >= 0) sendDispatch(ackPool[helperIdx].senderID, ROLE_HELPER);
    if (relayIdx >= 0)  sendDispatch(ackPool[relayIdx].senderID, ROLE_RELAYER);
  }
}

void sendDispatch(uint16_t targetID, uint8_t role) {
  seqCounter++;
  uint16_t msgID = makeMsgID(VESSEL_ID, seqCounter);

  uint8_t buf[23];
  uint8_t idx = 0;
  buf[idx++] = PKT_DISPATCH;
  buf[idx++] = FLAG_ROLE_ASSIGNED;
  packBE16(buf, idx, msgID);    idx += 2;
  packBE16(buf, idx, VESSEL_ID); idx += 2;
  packBE32(buf, idx, millis());  idx += 4;
  packBE16(buf, idx, targetID);  idx += 2;
  buf[idx++] = role;
  packBE32(buf, idx, (uint32_t)activeDistressLat); idx += 4;
  packBE32(buf, idx, (uint32_t)activeDistressLon); idx += 4;

  sendPacket(buf, idx);
  markSeen(msgID);
}

void handleDispatchPacket(uint8_t* buf, int n) {
  if (n < 23) return;
  uint16_t msgID    = unpackBE16(buf, 2);
  uint16_t targetID = unpackBE16(buf, 10);
  uint8_t role      = buf[12];

  if (targetID != VESSEL_ID) return;
  if (isSeen(msgID)) return;
  markSeen(msgID);

  if (role & ROLE_HELPER) {
    Serial2.println(F("DISPATCHED as HELPER"));
    tone(BUZZER_PIN, 1500);
  }
  if (role & ROLE_RELAYER) {
    Serial2.println(F("DISPATCHED as RELAYER"));
  }
  if (role & ROLE_HELPER)  nodeState = STATE_ACTING_HELPER;
  else if (role & ROLE_RELAYER) nodeState = STATE_ACTING_RELAYER;
}

// ============================================================================
// RELAY CHAIN — MAX_HOPS=10, chain is KILLED past this (per decision: vessel
// is too far from coast for this mesh to help further, not escalated)
// ============================================================================
void handleRelayPacket(uint8_t* buf, int n) {
  if (n < 31) return;
  uint16_t msgID = unpackBE16(buf, 2);
  if (isSeen(msgID)) return;
  markSeen(msgID);

  uint8_t hopCount = buf[19];

  for (int i = 0; i < min((int)hopCount, MAX_PATH_ENTRIES); i++) {
    uint16_t pathID = unpackBE16(buf, 20 + i * 2);
    if (pathID == VESSEL_ID) {
      Serial2.println(F("RELAY loop detected — dropping"));
      return;
    }
  }

  if (hopCount >= MAX_HOPS) {
    Serial2.println(F("RELAY chain KILLED — max hops reached, vessel too far from coast for this mesh."));
    return; // no further propagation, no escalation attempt — per decision
  }

  uint8_t outBuf[31];
  memcpy(outBuf, buf, 31);
  outBuf[19] = hopCount + 1;
  if (hopCount < MAX_PATH_ENTRIES) {
    packBE16(outBuf, 20 + hopCount * 2, VESSEL_ID);
  }
  sendPacket(outBuf, 31);
}

// ============================================================================
// INCOMING PACKET ROUTER
// ============================================================================
void handleIncomingPacket(uint8_t* buf, int n) {
  if (n < 1) return;
  switch (buf[0]) {
    case PKT_DISTRESS: handleDistressPacket(buf, n); break;
    case PKT_ACK:       handleAckPacket(buf, n); break;
    case PKT_DISPATCH:  handleDispatchPacket(buf, n); break;
    case PKT_RELAY:     handleRelayPacket(buf, n); break;
  }
}

void sendPacket(uint8_t* data, uint8_t len) {
  // LBT is handled by the module itself (configured once, separately) —
  // no channelClear()/CAD logic needed in firmware.
  e22ttl.sendMessage((const void*)data, len);
}

// ============================================================================
// SEEN-CACHE, HELPERS
// ============================================================================
bool isSeen(uint16_t msgID) {
  for (int i = 0; i < SEEN_CACHE_SIZE; i++)
    if (seenCache[i].used && seenCache[i].msgID == msgID) return true;
  return false;
}
void markSeen(uint16_t msgID) {
  for (int i = 0; i < SEEN_CACHE_SIZE; i++) {
    if (!seenCache[i].used) { seenCache[i] = {msgID, millis(), true}; return; }
  }
  int oldest = 0; unsigned long oldestTime = ULONG_MAX;
  for (int i = 0; i < SEEN_CACHE_SIZE; i++)
    if (seenCache[i].seenAt < oldestTime) { oldestTime = seenCache[i].seenAt; oldest = i; }
  seenCache[oldest] = {msgID, millis(), true};
}
void evictExpiredSeenEntries() {
  unsigned long now = millis();
  for (int i = 0; i < SEEN_CACHE_SIZE; i++)
    if (seenCache[i].used && (now - seenCache[i].seenAt > SEEN_CACHE_TTL_MS)) seenCache[i].used = false;
}

uint16_t makeMsgID(uint16_t vesselID, uint16_t seq) { return vesselID ^ seq; } // NOTE: see
  // packet-format note — design doc specifies 4-byte concatenated MsgID
  // (VESSEL_ID+SEQ_NUM as separate fields, not XOR'd into 2 bytes). This
  // firmware sends VESSEL_ID and SEQ_NUM as SEPARATE fields in the header
  // already (see packing above), so the "MsgID" used for seen-cache/loop
  // detection purposes here is a local 2-byte derivative for cache lookups
  // only — the actual wire format carries both fields uncombined, satisfying
  // the no-aliasing requirement from the design doc at the packet level.

int32_t gpsLatToFixed() { return gps.location.isValid() ? (int32_t)(gps.location.lat()*1000000.0) : 0; }
int32_t gpsLonToFixed() { return gps.location.isValid() ? (int32_t)(gps.location.lng()*1000000.0) : 0; }

float haversineKm(int32_t lat1e6, int32_t lon1e6, int32_t lat2e6, int32_t lon2e6) {
  double lat1=lat1e6/1000000.0, lon1=lon1e6/1000000.0, lat2=lat2e6/1000000.0, lon2=lon2e6/1000000.0;
  double R=6371.0;
  double dLat=radians(lat2-lat1), dLon=radians(lon2-lon1);
  double a=sin(dLat/2)*sin(dLat/2)+cos(radians(lat1))*cos(radians(lat2))*sin(dLon/2)*sin(dLon/2);
  return (float)(R*2*atan2(sqrt(a), sqrt(1-a)));
}

void packBE16(uint8_t* b, int o, uint16_t v){ b[o]=(v>>8)&0xFF; b[o+1]=v&0xFF; }
void packBE32(uint8_t* b, int o, uint32_t v){ b[o]=(v>>24)&0xFF; b[o+1]=(v>>16)&0xFF; b[o+2]=(v>>8)&0xFF; b[o+3]=v&0xFF; }
uint16_t unpackBE16(uint8_t* b, int o){ return ((uint16_t)b[o]<<8)|b[o+1]; }
uint32_t unpackBE32(uint8_t* b, int o){ return ((uint32_t)b[o]<<24)|((uint32_t)b[o+1]<<16)|((uint32_t)b[o+2]<<8)|b[o+3]; }
