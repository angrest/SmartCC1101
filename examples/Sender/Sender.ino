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
    // SPI communication failed. This is almost always a hardware problem:
    //   - CC1101 requires 3.3 V — use a level shifter with 5 V boards
    //   - Check VCC, GND, and all four SPI wires (SCK, MISO, MOSI, CS)
    //   - Cold solder joints and loose jumper wires are common culprits
    // Calling init() again will not help until the hardware issue is resolved.
    Serial.println(F("[E] CC1101 connection error — check wiring and power supply"));
    while (1);
  }
  Serial.println(F("[I] CC1101 connected."));

  Smartcc1101.setCarrierFrequency(868350000);  // 868.35 MHz
  Smartcc1101.setCRCCheck(true);
}

void loop() {
  // sendData() wakes the CC1101 automatically — no explicit wakeup call needed.
  if (Smartcc1101.sendData("Hello world!")) {
    Serial.println(F("Sent: Hello world!"));
  } else {
    Serial.print(F("[E] Send failed, error code: "));
    Serial.println(Smartcc1101.getLastError());
    // Error 1 (err_SPI_TIMEOUT): MISO never responded — hardware fault, check wiring.
    // Errors 2-4: may be transient. Try clearError() + init() once; if the problem
    // persists, inspect wiring and power before assuming it is a software issue.
    Smartcc1101.clearError();
    // Smartcc1101.init();  // uncomment to attempt recovery after a transient glitch
  }

  // Put the CC1101 into power-down mode (~200 nA) between transmissions.
  // In a real low-power application, also put the MCU to sleep here
  // (e.g. LowPower.powerDown() on AVR, esp_light_sleep_start() on ESP32).
  Smartcc1101.sleep();

  delay(1000);
}
