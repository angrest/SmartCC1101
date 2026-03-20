/**
* @brief ESP32 example using HSPI bus with custom pin mapping
* @file ESP32_HSPI_Sender.ino
* @author Axel Grewe
*
* Demonstrates how to use SmartCC1101 with a non-default SPI bus or
* custom pin mapping on ESP32. This is useful when:
*   - The default VSPI pins (18/19/23/5) are occupied by another peripheral
*   - You want to use HSPI (GPIO 14/12/13/15 by default)
*   - You need a different CS pin than the default
*
* ESP32 SPI buses:
*   VSPI (default SPI): SCK=18, MISO=19, MOSI=23, CS=5
*   HSPI:               SCK=14, MISO=12, MOSI=13, CS=15
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
#include <SPI.h>

// --- Pin definitions for HSPI bus ---
#define CC1101_SCK  14
#define CC1101_MISO 12
#define CC1101_MOSI 13
#define CC1101_CS   15

// Create a SPIClass instance on the HSPI bus.
// Use VSPI if you want the default bus but with different pins:
//   SPIClass cc1101SPI(VSPI);
SPIClass cc1101SPI(HSPI);

// Alternatively: use the default SPI object with a custom CS pin only:
//   Smartcc1101.init(7);  // CS on GPIO 7, all other pins stay at their defaults

void setup() {
  Serial.begin(115200);

  // Initialize CC1101 on HSPI with custom pins.
  // Parameter order: csPin, sckPin, cipoPin (MISO), copiPin (MOSI), SPIClass
  Smartcc1101.init(CC1101_CS, CC1101_SCK, CC1101_MISO, CC1101_MOSI, cc1101SPI);

  if (Smartcc1101.getCC1101()) {
    Serial.println(F("[I] CC1101 connected on HSPI."));
  } else {
    Serial.println(F("[E] *** CC1101 connection error — check wiring ***"));
    while (1);
  }

  // Optional: replace the internal delay() calls with a FreeRTOS-friendly
  // alternative so the scheduler can run other tasks during SPI waits.
  // Only useful when running under an RTOS — omit for standard Arduino projects.
  //
  //   Smartcc1101.setDelayFunction([](uint8_t ms) {
  //     vTaskDelay(ms / portTICK_PERIOD_MS);
  //   });

  Smartcc1101.setCarrierFrequency(868350000);              // 868.35 MHz
  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);
  Smartcc1101.setSymbolRate(100000);                       // 100 kBaud
  Smartcc1101.setDeviation(47608);
  Smartcc1101.setRXBandwidth(SmartCC1101::bw_812kHz);
  Smartcc1101.setSyncWord(0xD3, 0x91);
  Smartcc1101.setSyncMode(SmartCC1101::sync_1616);
  Smartcc1101.setLengthConfig(SmartCC1101::pktl_VARIABLE);
  Smartcc1101.setPacketLength(61);
  Smartcc1101.setCRCCheck(true);
  Smartcc1101.setPA(12);
}

void loop() {
  const char messageText[] = "Hello from ESP32 HSPI!";
  Smartcc1101.sendData(messageText);
  Serial.println(F("Sent: Hello from ESP32 HSPI!"));
  delay(1000);
}
