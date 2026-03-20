/**
 * @brief Low-power sensor transmitter example
 * @file LowPowerSensor.ino
 * @author Axel Grewe
 *
 * Demonstrates:
 *   - 433.92 MHz band operation
 *   - Sending binary data (struct) using sendData(uint8_t*, size)
 *   - Fixed packet length mode
 *   - sleep() to power down the CC1101 (~200 nA) between transmissions
 *     (wakeup is automatic on the next sendData(), setRX(), or receiveData() call;
 *      restored registers are handled transparently by the library)
 *
 * The packet payload is a small C struct containing a counter and simulated
 * sensor readings. Replace the placeholder values with real sensor reads.
 *
 * Pair with the LowPowerReceiver example.
 *
 * @note CC1101 payload is limited to 61 bytes. The struct size is verified
 *       at compile time.
 *
 * @copyright
 *  MIT License
 *
 *  Copyright (c) 2024 Axel Grewe
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include <SmartCC1101.h>

// ---------------------------------------------------------------------------
// Packet layout — must match LowPowerReceiver exactly.
// Using fixed-width types avoids size mismatches between different platforms.
// ---------------------------------------------------------------------------
struct SensorPacket {
  uint16_t counter;      // transmission counter (wraps at 65535)
  int16_t  temperature;  // temperature in 1/10 °C  (e.g. 235 = 23.5 °C)
  uint8_t  humidity;     // relative humidity 0–100 %
  uint8_t  battery;      // battery level 0–100 %
};

// Catch accidental struct growth at compile time.
static_assert(sizeof(SensorPacket) <= 61, "SensorPacket exceeds CC1101 payload limit of 61 bytes");

uint16_t txCounter = 0;

void setup() {
  Serial.begin(115200);

  Smartcc1101.init();

  if (!Smartcc1101.getCC1101()) {
    Serial.println(F("[E] CC1101 connection error — check wiring"));
    while (1);
  }
  Serial.println(F("[I] CC1101 connected."));

  // 433.92 MHz — common ISM band for low-power sensors.
  // The CC1101 handles the necessary register patches automatically
  // when switching between the 868 MHz and 433 MHz bands.
  Smartcc1101.setCarrierFrequency(433920000);

  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);

  // 4.8 kBaud with narrow deviation and filter: maximises receiver sensitivity
  // at the cost of throughput. Well suited for small, infrequent sensor packets.
  Smartcc1101.setSymbolRate(4800);
  Smartcc1101.setDeviation(5157);
  Smartcc1101.setRXBandwidth(SmartCC1101::bw_58kHz);

  Smartcc1101.setPA(10);               // 10 dBm — good outdoor range, moderate current
  Smartcc1101.setSyncWord(0xAB, 0xCD); // custom sync word — change to avoid interference
  Smartcc1101.setCRCCheck(true);

  // Fixed packet length: the receiver knows exactly how many bytes to expect,
  // which simplifies parsing and avoids a length-byte overhead in the payload.
  Smartcc1101.setLengthConfig(SmartCC1101::pktl_FIXED);
  Smartcc1101.setPacketLength(sizeof(SensorPacket));
}

void loop() {
  // --- Build the packet ---
  SensorPacket pkt;
  pkt.counter     = txCounter++;
  pkt.temperature = 235;  // 23.5 °C — replace with real sensor reading
  pkt.humidity    = 58;   // 58 %   — replace with real sensor reading
  pkt.battery     = 87;   // 87 %   — replace with ADC / battery monitor reading

  // --- Transmit ---
  // sendData() accepts a raw byte pointer and the number of bytes to send.
  // It wakes the CC1101 automatically — no explicit wakeup call needed.
  Smartcc1101.sendData(reinterpret_cast<const uint8_t *>(&pkt), sizeof(pkt));

  Serial.print(F("Sent packet #"));
  Serial.println(pkt.counter);

  // --- Power down the CC1101 (~200 nA) until the next transmission ---
  // The next sendData() call will wake it and restore all registers automatically.
  // In a real low-power application, also put the MCU to sleep here
  // (e.g. LowPower.powerDown() on AVR, esp_light_sleep_start() on ESP32).
  Smartcc1101.sleep();

  delay(5000);  // replace with MCU deep sleep for actual power savings
}
