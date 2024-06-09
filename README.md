# SmartCC1101
This driver library offers a simple access to data transfer using the **CC1101** module. It gives access to many configuration options without need to use RFStudio.
## Basic Function
This library was mainly implemented to efficiently send data from decentral battery powered sensors to a central hub. Howwever it may as well be used for bi-directional communication between several nodes.
The maximum amount of data which can be be sent in a single transfer is limited to the size of the internal buffer of the **CC1101**, leading to a possible payload of 61 bytes.
The code was designed to have a small memory footprint and runs fine on an Arduino Pro Mini (ATMega328p). However, for a single caclulation, floating point math is needed, excluding platforms like ATTiny. In a future version, I might re-implement the offending algortihm to integer math tpo remove the restriction. Besides that, no specific platform-dependant code is used and the library should behave well everywhere else (although it is only explicitely testetd on Arduino Pro Mini and ESP8266).
## Wiring
I promise I will add some drawings later on, but for now, I'll stick with the pin numbers - which are standard pins for SPI connections on the boards mentioneed.

### **CC1101**
If the module lays with the antenna connectors to the top, on the lower connection pins are from left to right:
+ VCC
+ GND
+ COPI (may also be called MOSI)
+ SCK (or SCLK)
+ CIPO (or MISO)
+ GDO2 (unused)
+ GDO0 (unused)
+ CS (or CSN)

The modules usually support voltages from 1.8V to 3.6V, so please use a level converter when connecting to a 5V CPU board. Also, be careful when soldering the connections, I have grilled several **CC1101** modules...

### Processor boards
**Arduino Pro Mini (ATMega328p)**
+ SCK 13
+ CIPO 12
+ COPI 11
+ CS 10

**ESP8266**
+ SCK 14
+ CIPO 12
+ COPI 13
+ CS 15

**ESP32**
+ SCK 18
+ CIPO 19
+ COPI 23
+ CS 5

**Arduino** 
+ SCK 13
+ CIPO 12
+ COPI 11
+ CS 10

For ESP32 and (to a limited extent) ESP8266 support custom SPI pinout. These differnt configurations need to be hardcoded in the library though. Later versions may support user-specified SPI configurations.

## Programming
The following code implements a simple sender and is available in the examples folder.

The settings are not limited to the `setup()` and may be changed anytime in the `loop()` if needed.

```C++

#include <SmartCC1101.h>

void setup() {
  Serial.begin(115200);

  SmartCC1101.init();  // must be called first to initialize the **CC1101**

  if (SmartCC1101.get**CC1101**()) {  // Check the **CC1101** SPI connection.
    Serial.println(F("[I] **CC1101** connected."));
  } else {
    Serial.println(F("[E] *** **CC1101** connection Error ***"));
    delay(20000);
  }

  SmartCC1101.setSymbolRate(100000);                        // Set the Data Rate in Baud. Value from 20 to 1621830 Default is 115051 Baud
  SmartCC1101.setManchester(true);                          // Enables Manchester encoding/decoding. false = Disable (default), true = Enable.
  SmartCC1101.setWhiteData(true);                           // Turn data whitening on / off. false = Whitening off (default). true = Whitening on.
  SmartCC1101.setCRCCheck(true);                            // true = CRC calculation in TX and CRC check in RX enabled. false = CRC disabled for TX and RX (default).
}

void loop() {

  const char messageText[] = "Hello world!";
  // Send data. Returns when data is actually sent.
  SmartCC1101.sendData(messageText);

  // Wait a moment untill next data sent.
  delay(1000);
}

```

The corresponding receiver (also available in the examples folder):

```C++

#include <SmartCC1101.h>

void setup() {
  Serial.begin(115200);

  SmartCC1101.init();  // must be called first to initialize the **CC1101**

  if (SmartCC1101.get**CC1101**()) {  // Check the **CC1101** SPI connection.
    Serial.println(F("[I] **CC1101** connected."));
  } else {
    Serial.println(F("[E] *** **CC1101** connection Error ***"));
    delay(20000);
  }

  SmartCC1101.setRXBandWitdth(SmartCC1101::bw_812kHz);      // Set the Receive Bandwidth in kHz. Possible values: bw_58kHz  = 58kHz, bw_68kHz, bw_81kHz, bw_102kHz,bw_116kHz, bw_135kHz, bw_162kHz, bw_203kHz (default), bw_232kHz, bw_270kHz, bw_325kHz, bw_406kHz, bw_464kHz, bw_541kHz, bw_650kHz, bw_812kHz = 812kHz
  SmartCC1101.setSymbolRate(100000);                        // Set the Data Rate in Baud. Value from 20 to 1621830 Default is 115051 Baud
  SmartCC1101.setManchester(true);                          // Enables Manchester encoding/decoding. false = Disable (default), true = Enable.
  SmartCC1101.setWhiteData(true);                           // Turn data whitening on / off. false = Whitening off (default). true = Whitening on.
  SmartCC1101.setCRCCheck(true);                            // true = CRC calculation in TX and CRC check in RX enabled. false = CRC disabled for TX and RX (default).
}


void loop() {
  uint8_t buffer[61]{ 0 };  // buffer for the data received by **CC1101**

  int len = SmartCC1101.receiveData(buffer);
  // len will be 0 if nothing is yet received.
  if (len > 0) {
    Serial.print(len);
    Serial.println(" bytes received.");

    // check transfer quality parameters
    int8_t RSSI = SmartCC1101.getRSSI();
    bool CRC = SmartCC1101.checkCRC();
    uint8_t LQI = SmartCC1101.getLQI();

    // will try to interpret the buffer received as character array, just make sure it's zero-terminated.
    // if it was actually sent from a character array, this is not necessary
    buffer[len] = 0;

    // check if crc is correct
    if (CRC) {
      Serial.println("CRC ok. RSSI: ");
      Serial.print(RSSI);
      Serial.print("dB, LQI ");
      Serial.println(LQI);
      Serial.println("Data:");
      Serial.println((char *)buffer);
    } else {
      Serial.print("CRC Error. RSSI: ");
      Serial.print(RSSI);
      Serial.print("dB, LQI ");
      Serial.println(LQI);
    }
  }
}

```

## Function reference

`void init(void)` must be called prior to using any other function. Will establish the necessary setup of the **CC1101** chip. Only exception: `getCC1101()` may be called prior to `init()`.

`bool checkCRC(void)` Check if CRC is correct. Returns `true` if CRCs match in last transmission. Only meaningful after data is received via `receiveData()`.

`bool getCC1101(void)` checks for the presence of a **CC1101** module on the SPI bus.

`uint8_t getLQI(void)` Link Quality Indicator. The Link Quality Indicator is a metric of the current quality of the received signal (smaller is better). When called after data is received via `receiveData()`. LQI corresponds to the data received. If called intermittently, returns the current link quality.

`int8_t getRSSI(void)` Received Signal Strength Indicator. The value is the estimated signal strength in dBm. When called after data is received via `receiveData()`. RSSI corresponds to the data received. If called intermittently, returns the current signal strength.

`uint8_t receiveData(uint8_t *rxBuffer)` Check if data is received and copy it to rxBuffer. Returns 0 if no data is received yet, transmission is in progress or CRC wrong and `void setCRC_AF(true)`. If data was received, `setRX()`must be called again.

`void sendData(const char *txBuffer)` Sends the zero-terminated character string pointed to. Will return when data is sent. Must be max 61 characters including the trailing 0.

`void sendData(const uint8_t *txBuffer, uint8_t size)` Sends size bytes pointed to by txBuffer. Must be max. 61 bytes.

`void setAddr(uint8_t addr)` On TX, sets the address of the package to be sent. On RX, determines from which address are acceptes if address check is enabled with `setAddrCheck`.

`void setAdrChk(AddressCheck adc)` Controls address check for received packages.
| Value | Address check configuration |
|------------|-------|
|   adc_NO | No address check (default) |
|  adc_YESNOBC | Address check, no broadcast |
|  adc_YES0BC | Address check and 0 (0x00) broadcast |
|  adc_YES0FFBC | Address check and 0 (0x00) and 255 (0xFF) broadcast |

`void setCarrierFrequency(uint32_t freq)` Set tranmsision frequency in Hz. Default = 868.35 MHz). The **CC1101** can use the frequenciy ranges 300-348 MHZ, 387-464MHZ and 779-928MHZ.
> [!WARNING]
> Please respect local regulations.

> [!IMPORTANT]
> Must match sender and receiver.

> [!NOTE]
> The effective carrier frequencies are quantized, the function will set it to the next possible lower value.

`void setCRCCheck(bool crcc)` On TX, enables CRC to be added to the payload. On RX, will calculate the CRC of received data and compare with CRC transmitted. Defaults to `false`.

`void setCRC_AF(bool af)` Autoflushes RX buffer if CRC check fails (if `setCRCCheck(true)`). Has no effect if `setCRCCheck(false)`.

`void setDCFilterOff(bool dcf)` Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud. The recommended IF frequency changes when the DC blocking is disabled. true = Disable (current optimized), false = Enable (better sensitivity).

`void setDelayFunction(void delayFunc(uint8_t))` The function supplied will be called repeatedly while waiting from the transmission to finish in `sendData()`. It must support a single parameter of type `uint8_t`, which is the delay in ms. If not set, `delay()` will be called. Should be needed only in very special caeses.

`void setDeviation(uint32_t deviation)` Set the frequency deviation in Hz. Possible values from 1586 to 380850. Default is 47607 Hz. 
> [!NOTE]
> the effective deviation values are quantized and algorithm rounds down. Using the exact interval limits, the next lower value might be taken (e.g. for 47607, you must actually specify 4760**8**).

`void setFEC(bool fec)` Enable forward error correction. Only supported in fixed packet length mode (`PacketLengthConfig(pktl_FIXED)`) and `setCRCCheck(true)`.

`void setManchester(bool menc)` Enable Manchester encoding. Defaults to `false`. 
> [!IMPORTANT]
> Must match sender and receiver.


`void setModulation(Modulation m)` Set modulation mode. I nearly exclusively use 2FSK modulation.
> [!IMPORTANT]
> Must match sender and receiver.

| Value | Modulation |
|------------|-------|
|mod_2FSK| 2FSK modulation (default)|
|mod_GFSK| GFSK modulation|
|mod_ASKOOK| ASK/OOK modulation (untested)|
|mod_4FSK| 4FSK modulation|
|mod_MSK| MFSK modulation|


`void setLengthConfig(PacketLengthConfig pktl)` From the different options the CC11101 supports in hardware, only the following two options are imlemented in this driver:
| Value | Packet length configuration |
|------------|-------|
|   pktl_FIXED | fixed packet length as configured by `setLengthConfig()` |
|   pktl_VARIABLE | Variable packet length |

 `void setPA(int8_t pa)` Set TXPower in dB. The following settings are possible depending on the frequency band: -30,  -20,  -15,  -10,  -6,    0,    5,    7,    10,   11,   12. Default is max. 
> [!WARNING]
> Please respect local regulations.

> [!NOTE]
> The effective PA values are quantized, the function will set it to the next possible lower value.

`void setPacketLength(uint8_t len)` Packet lentgh when `setLengthConfig(pktl_FIXED)`. On variable packet length, configures maximum packet length (optional). Defaults to 0.

  `void setPktFormat(PacketFormat pktf)` Although the **CC1101** supports different packet formats, only "normal" and RANDOM_TX are supported.
| Value | Packet format |
|------------|-------|
|    pktf_NORMAL  | Normal mode, use FIFOs for RX and TX (default)|
|   pktf_RANDOMTX | sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00) in RX |

`void setPRE(preamble_Bytes pre)` When enabling TX, the modulator will start transmitting the preamble. When the programmed number of preamble bytes has been transmitted, the modulator will send the sync word and then data from the TX FIFO.  
> [!IMPORTANT]
> Must match sender and receiver.

| Value | Preamble Bytes |
|------------|-------|
|    pre_2  |2 preamble bytes |
|    pre_3  | 3 preamble bytes |
|    pre_4  | 4 preamble bytes (default) |
|    pre_6  | 6 preamble bytes |
|    pre_8  | 8 preamble bytes |
|    pre_12  | 12 preamble bytes |
|    pre_16  | 16 preamble bytes |
|    pre_24 = |/ 24 preamble bytes |


`void setPQT(uint8_t pqt)` The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by eight each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. By setting the value to zero, the preamble quality qualifier of the sync word is disabled. Defaults to 0.

`void setRX(void)` Set the **CC1101** to RX mode. Repeated calls do not have any effect, unless data was received in the meantime. If the internal buffer is not read via `receiveData()`, the data is lost.

`void setRXBandWitdth(rx_BandWidth bw)` Set the Receive Bandwidth. Value can be cosen from predefined list of values:   
| Bandwidth | Value   |
|----------|---------|
| 58KHz | bw_58kHz |
| 68kHz | bw_68kHz |
| 81kHz | bw_81kHz |
| 102kHz | bw_102kHz |
| 116kHz | bw_116kHz |
| 135kHz | bw_135kHz |
| 203kHz | bw_203kHz (default) |
| 232kHz | bw_232kHz |
| 270kHz | bw_270kHz |
| 325kHz | bw_325kHz |
| 406kHz | bw_406kHz |
| 464kHz | bw_464kHz |
| 541kHz | bw_541kHz |
| 650kHz | bw_650kHz |
| 812kHz | bw_812kHz |

`void setSymbolRate(double symbolRate)` Data transmission rate in baud. Value from 20 to 1621830 Default is 115051 Baud. 
> [!IMPORTANT]
> Must match on sender and receiver.

> [!NOTE]
> The effective symbol rates are quantized, the function will set it to the next possible lower value.
  
`void setSyncMode(sync_Mode syncm)` Combined sync-word qualifier mode.  The hardware supports carrier sense with abolute and relative thresholds as well, but this is not implemented.
>[!IMPORTANT]
> `sync_3032` enables duplicate transmission of the sync word in TX. This must match the setting in RX, so chose `sync_3032` here as well.

| Value | Sync-Mode |
|------------|-------|
|    sync_NONE | No preamble/sync |
|    sync_1516 | 15/16 sync word bits detected |
|    sync_1616 | 16/16 sync word bits detected (default)|
|    sync_3032 | 30/32 sync word bits detected |

`void setSyncWord(uint8_t sh, uint8_t sl)` If sync-word detection is enabled via setSyncMode,the **CC1101** will not start filling the RX FIFO unless a valid sync word is detected. 
> [!IMPORTANT]
> Must match on sender and receiver.

`void setWhiteData(bool white)` Enable data whitening. Defaults to `false`. 
> [!IMPORTANT]
> Must match on sender and receiver.

`void sleep(void)` Sets the **CC1101** to sleep mode, reducing power consumption. The next access will wake the chip up again, no special procedure is needed.


