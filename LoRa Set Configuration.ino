/*
 * LoRa E22-900T30D — ONE-TIME CONFIGURATION SKETCH
 * Debug output goes over Serial1 (PA9=TX, PA10=RX) to your USB-TTL adapter,
 * since you're flashing via ST-Link and the native USB isn't used for this.
 *
 * WIRING (config mode — M1 must be HIGH during this):
 * E22        ----- STM32 Blue Pill
 * M0         ----- PB0   (LOW during config, LOW again after — normal op is M0=0,M1=0)
 * M1         ----- PB10  (HIGH during config only — move to GND after config is verified)
 * TX         ----- PA3 (Serial2 RX)
 * RX         ----- PA2 (Serial2 TX)
 * AUX        ----- PA0
 * VCC        ----- 3.3V/5V
 * GND        ----- GND
 *
 * TTL ADAPTER (for Serial1 debug output):
 * Adapter GND ----- Blue Pill GND
 * Adapter RX  ----- Blue Pill PA9  (Serial1 TX)
 * Adapter TX  ----- Blue Pill PA10 (Serial1 RX)
 *
 * PROCEDURE PER VESSEL:
 * 1. Wire as above with M1 = HIGH (config mode).
 * 2. Flash via ST-Link, open Serial Monitor on the TTL adapter's port at 9600 baud.
 * 3. Confirm the AFTER printout shows: Chan=16, Power=30dBm-class, LBT=Enabled.
 * 4. Power off, move M1 to GND, power back on — module now runs in
 *    transparent/normal mode permanently with this saved config.
 * 5. Repeat for the next vessel's module.
 */

#define FREQUENCY_868   // Must be defined BEFORE the include — sets the 850-930MHz channel/frequency math for this band
#define E22_30          // Must be defined BEFORE the include — unlocks the 30dBm-class power constants

#include "Arduino.h"
#include "LoRa_E22.h"

// NOTE: Serial2 is NOT declared here — this core (generic_stm32f103c board
// variant) already predefines a global Serial2 object internally, mapped to
// the default USART2 pins (PA2=TX, PA3=RX), which is exactly what we want.
// Declaring our own caused a linker error: "multiple definition of Serial2".
LoRa_E22 e22ttl(&Serial2, PA0, PB0, PB10); // Serial, AUX, M0, M1

void printParameters(struct Configuration configuration);

void setup() {
  Serial1.begin(9600);
  delay(500);

  Serial1.println();
  Serial1.println(F("=== LoRa E22 ONE-TIME CONFIG ==="));

  e22ttl.begin();

  // ---- READ CURRENT CONFIG FIRST (never build a config struct from scratch) ----
  ResponseStructContainer c;
  c = e22ttl.getConfiguration();
  Configuration configuration = *(Configuration*) c.data;
  Serial1.println(F("--- BEFORE ---"));
  Serial1.println(c.status.getResponseDescription());
  printParameters(configuration);

  // ---- APPLY MESH-SHARED SETTINGS ----
  // These four must be IDENTICAL on every vessel — this is the shared
  // "network" address in transparent mode, not a per-device ID.
  configuration.ADDL = 0x00;
  configuration.ADDH = 0x00;
  configuration.NETID = 0x00;
  configuration.CHAN = 16;                 // 850.125 + 16 = 866.125 MHz — inside India's 865-867MHz WPC band

  configuration.SPED.uartBaudRate = UART_BPS_9600;
  configuration.SPED.airDataRate = AIR_DATA_RATE_010_24;
  configuration.SPED.uartParity = MODE_00_8N1;

  configuration.OPTION.subPacketSetting = SPS_240_00;
  configuration.OPTION.RSSIAmbientNoise = RSSI_AMBIENT_NOISE_DISABLED;
  configuration.OPTION.transmissionPower = POWER_30;   // <-- CHECK AUTOCOMPLETE: confirm this exact constant exists
                                                        //     for your library version once E22_30 is defined.
                                                        //     If it doesn't autocomplete, check the dropdown list
                                                        //     of OPTION.transmissionPower options in your IDE and
                                                        //     use whichever one corresponds to max/30dBm.

  configuration.TRANSMISSION_MODE.enableRSSI = RSSI_DISABLED;
  configuration.TRANSMISSION_MODE.fixedTransmission = FT_TRANSPARENT_TRANSMISSION;  // transparent, not fixed-address
  configuration.TRANSMISSION_MODE.enableRepeater = REPEATER_DISABLED;
  configuration.TRANSMISSION_MODE.enableLBT = LBT_ENABLED;   // <-- the important flip from the stock example
  configuration.TRANSMISSION_MODE.WORTransceiverControl = WOR_RECEIVER;
  configuration.TRANSMISSION_MODE.WORPeriod = WOR_2000_011;

  // ---- WRITE IT, PERMANENTLY (survives power-off) ----
  ResponseStatus rs = e22ttl.setConfiguration(configuration, WRITE_CFG_PWR_DWN_SAVE);
  Serial1.println(F("--- WRITE RESULT ---"));
  Serial1.println(rs.getResponseDescription());
  Serial1.println(rs.code);

  // ---- READ BACK AND VERIFY — do not trust a "success" write alone ----
  c = e22ttl.getConfiguration();
  configuration = *(Configuration*) c.data;
  Serial1.println(F("--- AFTER (verify these match what you set) ---"));
  Serial1.println(c.status.getResponseDescription());
  printParameters(configuration);

  Serial1.println(F("=== DONE. If Chan/Power/LBT above match, power off, "));
  Serial1.println(F("    move M1 to GND, and this module is ready. ==="));
}

void loop() {
  // Nothing — this sketch only runs once at setup(). Re-flash your real
  // firmware after configuration is confirmed.
}

void printParameters(struct Configuration configuration) {
  Serial1.println(F("----------------------------------------"));
  Serial1.print(F("AddH : "));  Serial1.println(configuration.ADDH, HEX);
  Serial1.print(F("AddL : "));  Serial1.println(configuration.ADDL, HEX);
  Serial1.print(F("NetID : "));  Serial1.println(configuration.NETID, HEX);
  Serial1.print(F("Chan : "));  Serial1.print(configuration.CHAN, DEC); Serial1.print(F(" -> ")); Serial1.println(configuration.getChannelDescription());
  Serial1.print(F("SpeedUARTBaud    : ")); Serial1.println(configuration.SPED.getUARTBaudRateDescription());
  Serial1.print(F("SpeedAirDataRate : ")); Serial1.println(configuration.SPED.getAirDataRateDescription());
  Serial1.print(F("OptionTranPower  : ")); Serial1.println(configuration.OPTION.getTransmissionPowerDescription());
  Serial1.print(F("TransModeFixedTrans: ")); Serial1.println(configuration.TRANSMISSION_MODE.getFixedTransmissionDescription());
  Serial1.print(F("TransModeEnableLBT : ")); Serial1.println(configuration.TRANSMISSION_MODE.getLBTEnableByteDescription());
  Serial1.println(F("----------------------------------------"));
}