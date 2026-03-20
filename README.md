# SmartCC1101

[![Arduino Lint](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml/badge.svg)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml)
[![Compile (Uno)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml/badge.svg?job=compile-avr)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml)
[![Compile (ESP32)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml/badge.svg?job=compile-esp32)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml)
[![Compile (ESP8266)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml/badge.svg?job=compile-esp8266)](https://github.com/angrest/SmartCC1101/actions/workflows/arduino-checks.yml)

Arduino library for the CC1101/CC1100 sub-GHz RF transceiver module.

Provides straightforward access to sending and receiving data, with full control over the CC1101's configuration registers — without needing TI's SmartRF Studio.

## Features

- Supports 300–348 MHz, 387–464 MHz, and 779–928 MHz bands (868 MHz tested; 433 MHz supported in hardware but not yet field-tested)
- Automatic band-specific register patching when switching between 868 MHz and 433 MHz
- Configurable modulation (2-FSK, GFSK, ASK/OOK, 4-FSK, MSK)
- Variable or fixed packet length mode (max. 61 bytes payload)
- CRC check, data whitening, Manchester encoding, FEC
- Low memory footprint — runs on Arduino Nano/Pro Mini
- No floating point operations and no `<stdio.h>` formatted I/O (`printf`/`sprintf`) — safe for AVR targets with limited flash
- Custom delay function support (e.g. for RTOS integration)

## Supported Boards

| Architecture | Example boards |
|---|---|
| AVR | Arduino Uno, Nano, Pro Mini, Mega |
| ESP8266 | NodeMCU, Wemos D1 |
| ESP32 | ESP32 DevKit |

## Installation

### Arduino Library Manager (recommended)
1. Open Arduino IDE → **Sketch → Include Library → Manage Libraries**
2. Search for `SmartCC1101`
3. Click **Install**

### Manual
1. Download the [latest release](https://github.com/angrest/SmartCC1101/releases)
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library**

## Wiring (SPI)

| CC1101 | Arduino Uno/Nano | ESP8266 | ESP32 |
|--------|-----------------|---------|-------|
| VCC    | 3.3V            | 3.3V    | 3.3V  |
| GND    | GND             | GND     | GND   |
| SCK    | D13             | D14     | D18   |
| MISO   | D12             | D12     | D19   |
| MOSI   | D11             | D13     | D23   |
| CSN    | D10             | D15     | D5    |

> **Note:** The CC1101 is a 3.3V device. Use a level shifter when connecting to 5V boards.

## Quick Start

```cpp
#include <SmartCC1101.h>

void setup() {
  Smartcc1101.init();

  if (!Smartcc1101.getCC1101()) {
    // SPI connection failed — check wiring
    while (1);
  }

  Smartcc1101.setCarrierFrequency(868350000);  // 868.35 MHz
  Smartcc1101.setCRCCheck(true);
}

// Sender
void loop() {
  Smartcc1101.sendData("Hello world!");
  delay(1000);
}

// Receiver
void loop() {
  uint8_t buffer[61];
  uint8_t len = Smartcc1101.receiveData(buffer);
  if (len > 0) {
    buffer[len] = 0;  // null-terminate for use as string
    Serial.println((char *)buffer);
  }
}
```

See `examples/Sender` and `examples/Receiver` for full examples.

## API Overview

### Initialization
| Function | Description |
|---|---|
| `init(csPin, sckPin, cipoPin, copiPin, spi)` | Initialize SPI and reset CC1101. All parameters optional, defaults to platform-specific pins and the default `SPI` bus. Pass a custom `SPIClass` instance to use a different SPI bus (e.g. `HSPI` on ESP32). On AVR, only `csPin` is configurable — SPI pins are hardware-fixed. |
| `getCC1101()` | Returns `true` if CC1101 is reachable via SPI. |

### Radio Configuration
| Function | Description |
|---|---|
| `setCarrierFrequency(uint32_t hz)` | Frequency in Hz. Supported: 300–348, 387–464, 779–928 MHz. |
| `setModulation(Modulation m)` | `mod_2FSK`, `mod_GFSK`, `mod_ASKOOK`, `mod_4FSK`, `mod_MSK` |
| `setPA(int8_t dBm)` | TX power: -30, -20, -15, -10, -6, 0, 5, 7, 10, 11, 12 dBm |
| `setRXBandwidth(rx_BandWidth bw)` | RX filter bandwidth: `bw_58kHz` … `bw_812kHz` |
| `setSymbolRate(uint32_t baud)` | Data rate: 20 – 1,621,830 Baud |
| `setDeviation(uint32_t hz)` | FSK frequency deviation: 1586 – 380,850 Hz |

### Packet Configuration
| Function | Description |
|---|---|
| `setSyncWord(uint8_t h, uint8_t l)` | Sync word (must match sender and receiver) |
| `setSyncMode(sync_Mode m)` | Sync word detection mode |
| `setPRE(preamble_Bytes p)` | Number of preamble bytes |
| `setLengthConfig(PacketLengthConfig c)` | `pktl_FIXED` or `pktl_VARIABLE` |
| `setPacketLength(uint8_t len)` | Max/fixed packet length (max. 61) |
| `setCRCCheck(bool)` | Enable CRC |
| `setCRC_AF(bool)` | Auto-flush RX FIFO on CRC error |
| `setManchester(bool)` | Manchester encoding |
| `setWhiteData(bool)` | Data whitening |
| `setFEC(bool)` | Forward Error Correction (fixed length only) |

### Send / Receive
| Function | Description |
|---|---|
| `sendData(const char*)` | Send null-terminated string (max. 61 bytes, must not be NULL). Returns `false` on error. |
| `sendData(const uint8_t*, uint8_t)` | Send byte array. Returns `false` on error. |
| `setRX()` | Switch to RX mode. Returns `false` on error. |
| `receiveData(uint8_t*)` | Read received data. Buffer must be ≥ 61 bytes. Returns number of bytes received. |
| `getRSSI()` | RSSI in dBm (typically -140 to 0) |
| `getLQI()` | Link Quality Indicator 0–127 (lower is better) |
| `checkCRC()` | `true` if last received packet had valid CRC |

### Error Handling
| Function | Description |
|---|---|
| `getLastError()` | Returns the last `ErrorCode`: `err_NONE`, `err_SPI_TIMEOUT`, `err_IDLE_TIMEOUT`, `err_TX_TIMEOUT`, `err_CALIB_TIMEOUT` |
| `clearError()` | Clears the error flag before retrying |

`sendData()` and `setRX()` return `false` on failure and set an error code. The library does **not** attempt automatic recovery.

`err_SPI_TIMEOUT` indicates that MISO did not respond — this points to a hardware problem (loose connection, cold solder joint, missing power). Calling `init()` will not fix this; inspect the wiring first. `err_IDLE_TIMEOUT`, `err_TX_TIMEOUT`, and `err_CALIB_TIMEOUT` can in principle occur after a transient glitch; calling `clearError()` followed by `init()` is worth trying, but a hardware fault cannot be ruled out.

Discarding the return value of `sendData()` / `setRX()` is valid C++ — existing code requires no changes.

### Power Management
| Function | Description |
|---|---|
| `sleep()` | Enter power-down mode. Wake-up is automatic on the next `sendData()` or `setRX()` call. |

## Known Limitations

- GDO pin-based communication is not implemented
- Maximum payload per transmission: 61 bytes
- Infinite packet length mode is not supported
- 315 MHz band register values are not patched (SmartRF Studio values needed)

## Crystal Frequency

Most CC1101 modules use a **26 MHz** crystal (default). Some variants use **27 MHz**.
Override at compile time if needed:

```cpp
// platformio.ini
build_flags = -DCC1101_CRYSTAL_FREQUENCY=27000000ul
```

## License

MIT License — Copyright (c) 2024 Axel Grewe
