/**
 * @brief Low-power sensor receiver example
 * @file LowPowerReceiver.ino
 * @author Axel Grewe
 *
 * Demonstrates:
 *   - 433.92 MHz band operation
 *   - Receiving binary data (struct) using receiveData()
 *   - Fixed packet length mode
 *   - CRC auto-flush (setCRC_AF) to discard corrupted packets automatically
 *   - RSSI and LQI link quality indicators
 *
 * Decodes the SensorPacket payload sent by the LowPowerSensor example.
 * The struct definition must match the sender exactly.
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
// Packet layout — must match LowPowerSensor exactly.
// ---------------------------------------------------------------------------
struct SensorPacket {
  uint16_t counter;      // transmission counter
  int16_t  temperature;  // temperature in 1/10 °C  (e.g. 235 = 23.5 °C)
  uint8_t  humidity;     // relative humidity 0–100 %
  uint8_t  battery;      // battery level 0–100 %
};

void setup() {
  Serial.begin(115200);

  Smartcc1101.init();

  if (!Smartcc1101.getCC1101()) {
    Serial.println(F("[E] CC1101 connection error — check wiring"));
    while (1);
  }
  Serial.println(F("[I] CC1101 connected."));

  // Settings must mirror the LowPowerSensor sketch exactly.
  Smartcc1101.setCarrierFrequency(433920000);
  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);
  Smartcc1101.setSymbolRate(4800);
  Smartcc1101.setDeviation(5157);
  Smartcc1101.setRXBandwidth(SmartCC1101::bw_58kHz);
  Smartcc1101.setSyncWord(0xAB, 0xCD);
  Smartcc1101.setCRCCheck(true);

  // Auto-flush the RX FIFO when CRC fails: receiveData() returns 0
  // for corrupted packets, so no manual CRC check is needed in loop().
  Smartcc1101.setCRC_AF(true);

  Smartcc1101.setLengthConfig(SmartCC1101::pktl_FIXED);
  Smartcc1101.setPacketLength(sizeof(SensorPacket));

  Smartcc1101.setRX();  // start listening
  Serial.println(F("[I] Listening on 433.92 MHz..."));
}

void loop() {
  uint8_t buffer[61];
  uint8_t len = Smartcc1101.receiveData(buffer);

  if (len == 0) return;  // nothing received (or CRC_AF discarded the packet)

  // Sanity check: with fixed packet length this should always hold, but
  // guard against unexpected states at startup.
  if (len != sizeof(SensorPacket)) {
    Serial.print(F("[W] Unexpected packet size: "));
    Serial.println(len);
    Smartcc1101.setRX();
    return;
  }

  // Copy raw bytes into the struct.
  // memcpy avoids strict-aliasing undefined behaviour compared to casting.
  SensorPacket pkt;
  memcpy(&pkt, buffer, sizeof(pkt));

  // Print decoded sensor values.
  // Temperature is stored in 1/10 °C, so divide by 10 for the integer part
  // and use the remainder for the fractional digit.
  Serial.print(F("Packet #"));
  Serial.print(pkt.counter);
  Serial.print(F("  Temp: "));
  Serial.print(pkt.temperature / 10);
  Serial.print(F("."));
  Serial.print(abs(pkt.temperature % 10));
  Serial.print(F(" C  Hum: "));
  Serial.print(pkt.humidity);
  Serial.print(F("%  Bat: "));
  Serial.print(pkt.battery);
  Serial.print(F("%  RSSI: "));
  Serial.print(Smartcc1101.getRSSI());
  Serial.print(F(" dBm  LQI: "));
  Serial.println(Smartcc1101.getLQI());

  Smartcc1101.setRX();  // go back to receive mode for next packet
}
