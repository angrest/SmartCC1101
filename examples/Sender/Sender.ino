/**
 * @brief Basic sender example for the SmartCC1101 library
 * @file Sender.ino
 * @author Axel Grewe
 *
 * Sends a text string once per second on 868.35 MHz.
 * Works on Arduino Uno/Nano/Mega, ESP8266, and ESP32 without modification.
 * Pair with the Receiver example.
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
}

void loop() {
  // sendData() wakes the CC1101 automatically if sleep() was called before.
  Smartcc1101.sendData("Hello world!");
  Serial.println(F("Sent: Hello world!"));

  // Put the CC1101 into power-down mode between transmissions.
  // In a real low-power application, also sleep the MCU here.
  Smartcc1101.sleep();

  delay(1000);
}
