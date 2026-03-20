/**
 * @brief Basic receiver example for the SmartCC1101 library
 * @file Receiver.ino
 * @author Axel Grewe
 *
 * Receives text packets on 868.35 MHz and prints them to Serial.
 * Works on Arduino Uno/Nano/Mega, ESP8266, and ESP32 without modification.
 * Pair with the Sender example.
 *
 * Wiring (default SPI pins):
 *   Arduino Uno/Nano: CS=10, SCK=13, MISO=12, MOSI=11
 *   ESP8266:          CS=15, SCK=14, MISO=12, MOSI=13
 *   ESP32:            CS=5,  SCK=18, MISO=19, MOSI=23
 *
 * To use a different CS pin:  Smartcc1101.init(7);
 * ESP32/ESP8266 custom pins:  Smartcc1101.init(CS, SCK, MISO, MOSI);
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

void setup() {
  Serial.begin(115200);

  Smartcc1101.init();

  if (!Smartcc1101.getCC1101()) {
    Serial.println(F("[E] CC1101 connection error — check wiring"));
    while (1);
  }
  Serial.println(F("[I] CC1101 connected."));

  Smartcc1101.setCarrierFrequency(868350000);  // 868.35 MHz
  Smartcc1101.setCRCCheck(true);

  if (!Smartcc1101.setRX()) {
    Serial.print(F("[E] setRX failed, error code: "));
    Serial.println(Smartcc1101.getLastError());
    while (1);
  }
  Serial.println(F("[I] Listening on 868.35 MHz..."));
}

void loop() {
  uint8_t buffer[61];
  uint8_t len = Smartcc1101.receiveData(buffer);

  if (len == 0) return;  // nothing received yet

  buffer[len] = 0;  // null-terminate for use as string

  Serial.print(F("Received ("));
  Serial.print(len);
  Serial.print(F(" bytes, RSSI "));
  Serial.print(Smartcc1101.getRSSI());
  Serial.print(F(" dBm, LQI "));
  Serial.print(Smartcc1101.getLQI());
  Serial.print(F("): "));
  Serial.println((char *)buffer);

  if (!Smartcc1101.setRX()) {
    Serial.print(F("[E] setRX failed, error code: "));
    Serial.println(Smartcc1101.getLastError());
    Smartcc1101.clearError();
  }
}
