# SmartCC1101
This driver library offers a simple access to data transfer using the cc1101 module. It gives access to many configuration options without need to use RFStudio.
## Basic Function
This library was mainly implemented to efficiently send data from decentral battery powered sensors to a central hub. Howwever it may as well be used for bi-directional communication between several nodes.
The maximum amount of data which can be be sent in a single transfer is limited to the size of the internal buffer of the CC1101, leading to a possible payload of 61 bytes.
The code was designed to have a small memory footprint and runs fine on an Arduino Pro Mini (ATMega328p). However, for a single caclulation, floating point math is needed, excluding platforms like ATTiny. In a future version, I might re-implement the offending algortihm to integer math tpo remove the restriction. Besides that, no specific platform-dependant code is used and the library should behave well everywhere else (although it is only explicitely testetd on Arduino Pro Mini and ESP8266).
## Wiring
I promise I will add some drawings later on, but for now, I'll stick with the pin numbers - which are standard pins for SPI connections on the boards mentioneed.

### CC1101
If the module lays with the antenna connectors to the top, on the lower connection pins are from left to right:
+ VCC
+ GND
+ COPI (may also be called MOSI)
+ SCK (or SCLK)
+ CIPO (or MISO)
+ GDO2 (unused)
+ GDO0 (unused)
+ CS (or CSN)

The modules usually support voltages from 1.8V to 3.6V, so please use a level converter when connecting to a 5V CPU board. Also, be careful when soldering the connections, I have grilled several CC1101 modules...

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
**Please note:** Nearly all settings are redundant and just replicate the default values (see comments). They are shown for illustration. Settings where sender and receiver need to match are marked in the comments.

The settings are not limited to the `setup()` and may be changed anytime in the `loop()` if needed.

```C++

#include <SmartCC1101.h>

void setup() {
  Serial.begin(115200);

  Smartcc1101.init();  // must be called first to initialize the CC1101

  if (Smartcc1101.getCC1101()) {  // Check the CC1101 SPI connection.
    Serial.println(F("[I] CC1101 connected."));
  } else {
    Serial.println(F("[E] *** CC1101 connection Error ***"));
    delay(20000);
  }

  // Most of the settings repeat standard settings and are here for illustration
  Smartcc1101.setDCFilterOff(false);                        // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud. The recommended IF frequency changes when the DC blocking is disabled. true = Disable (current optimized), false = Enable (better sensitivity).
  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);         // set modulation mode. Possible values: mod_2FSK (default), mod_GFSK, mod_ASKOOK, mod_4FSK, mod_MSK
  Smartcc1101.setCarrierFrequency(868350000);               // Set tranmsision frequency in Hz. Default = 868.35 MHz). The cc1101 can use 300-348 MHZ, 387-464MHZ and 779-928MHZ. More info in the datasheet.
  Smartcc1101.setPA(12);                                    // Set TXPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  Smartcc1101.setDeviation(47608);                          // Set the Frequency deviation in Hz. Value from 1586 to 380850. Default is 47607 kHz. Note: the algorithm rounds down. On exact interval limits, the next lower value might be taken (that's why here 47608 is used)
  Smartcc1101.setRXBandWitdth(SmartCC1101::bw_812kHz);      // Set the Receive Bandwidth in kHz. Possible values: bw_58kHz  = 58kHz, bw_68kHz, bw_81kHz, bw_102kHz,bw_116kHz, bw_135kHz, bw_162kHz, bw_203kHz (default), bw_232kHz, bw_270kHz, bw_325kHz, bw_406kHz, bw_464kHz, bw_541kHz, bw_650kHz, bw_812kHz = 812kHz
  Smartcc1101.setSymbolRate(100000);                        // Set the Data Rate in Baud. Value from 20 to 1621830 Default is 115051 Baud
  Smartcc1101.setSyncWord(0xD3, 0x91);                      // Set sync word. Must match in sender and receiver. (Syncword high, Syncword low). Values given are default values
  Smartcc1101.setSyncMode(SmartCC1101::sync_1616);          // Combined sync-word qualifier mode. sync_NONE = No preamble/sync. sync_1516 = 15/16 sync word bits detected. sync_1616 = 16/16 sync word bits detected (default). sync_3032 = 30/32 sync word bits detected (sync word sent twice).
                                                            // sync_NONECS = No preamble/sync, carrier-sense above threshold. sync_1516CS = 15/16 + carrier-sense above threshold. sync_1516CS = 15/16 + carrier-sense above threshold. sync_1616CS = 16/16 + carrier-sense above threshold. sync_3032CS = 30/32 (sync word sent twice). + carrier-sense above threshold.
  Smartcc1101.setPRE(SmartCC1101::pre_4);                   // Sets the minimum number of preamble bytes to be transmitted. Possible values: pre_2 : 2, pre_3 : 3, pre_4 : 4 (default), pre_6 : 6, pre_8 : 8, pre_12 : 12, pre_16 : 16, pre_24 : 24
  Smartcc1101.setPQT(0);                                    // Preamble quality estimator threshold. 0 is the derfault. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
  Smartcc1101.setAdrChk(SmartCC1101::adc_NO);               // Controls address check configuration of received packages. adc_NO = No address check (default). adc_YESNOBC = Address check, no broadcast. adc_YES0BC = Address check and 0 (0x00) broadcast. adc_YES0FFBC = Address check and 0 (0x00) and 255 (0xFF) broadcast.
  Smartcc1101.setAddr(0);                                   // Address used for packet filtration. 0 is default. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
  Smartcc1101.setManchester(true);                          // Enables Manchester encoding/decoding. false = Disable (default), true = Enable.
  Smartcc1101.setWhiteData(true);                           // Turn data whitening on / off. false = Whitening off (default). true = Whitening on.
  Smartcc1101.setPktFormat(SmartCC1101::pktf_NORMAL);       // Format of RX and TX data. pktf_NORMAL = Normal mode, use FIFOs for RX and TX (default). pktf_RANDOMTX = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX.
  Smartcc1101.setLengthConfig(SmartCC1101::pktl_VARIABLE);  // pktl_FIXED = Fixed packet length mode. pktl_VARIABLE = Variable packet length mode (default).
  Smartcc1101.setPacketLength(61);                          // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed. 61 is maximum allowed value in this implementation
  Smartcc1101.setFEC(false);                                // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. false = Disable (default), true = Enable.
  Smartcc1101.setCRCCheck(true);                            // true = CRC calculation in TX and CRC check in RX enabled. false = CRC disabled for TX and RX (default).
  Smartcc1101.setCRC_AF(false);                             // Enable automatic flush of RX FIFO when CRC is not OK. (Default: false)
}

void loop() {

  const char messageText[] = "Hello world!";
  // Send data. Returns when data is actually sent.
  Smartcc1101.sendData(messageText);

  // Wait a moment untill next data sent.
  delay(1000);
}

```

The Corresponding receiver is also available:

```C++

#include <SmartCC1101.h>

void setup() {
  Serial.begin(115200);

  Smartcc1101.init();  // must be called first to initialize the CC1101

  if (Smartcc1101.getCC1101()) {  // Check the CC1101 SPI connection.
    Serial.println(F("[I] CC1101 connected."));
  } else {
    Serial.println(F("[E] *** CC1101 connection Error ***"));
    delay(20000);
  }

  // Most of the settings repeat standard settings and are here for illustration 
  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);         // set modulation mode. Possible values: mod_2FSK (default), mod_GFSK, mod_ASKOOK, mod_4FSK, mod_MSK
  Smartcc1101.setCarrierFrequency(868350000);               // Set tranmsision frequency in Hz. Default = 868.35 MHz). The cc1101 can use 300-348 MHZ, 387-464MHZ and 779-928MHZ. More info in the datasheet.
  Smartcc1101.setDeviation(47608);                          // Set the Frequency deviation in Hz. Value from 1586 to 380850. Default is 47607 kHz. Note: the algorithm rounds down. On exact interval limits, the next lower value might be taken (that's why here 47608 is used)
  Smartcc1101.setRXBandWitdth(SmartCC1101::bw_812kHz);      // Set the Receive Bandwidth in kHz. Possible values: bw_58kHz  = 58kHz, bw_68kHz, bw_81kHz, bw_102kHz,bw_116kHz, bw_135kHz, bw_162kHz, bw_203kHz (default), bw_232kHz, bw_270kHz, bw_325kHz, bw_406kHz, bw_464kHz, bw_541kHz, bw_650kHz, bw_812kHz = 812kHz
  Smartcc1101.setSymbolRate(100000);                        // Set the Data Rate in Baud. Value from 20 to 1621830 Default is 115051 Baud
  Smartcc1101.setSyncWord(0xD3,0x91);                       // Set sync word. Must match in sender and receiver. (Syncword high, Syncword low). Values given are default values
  Smartcc1101.setSyncMode(SmartCC1101::sync_1616);          // Combined sync-word qualifier mode. sync_NONE = No preamble/sync. sync_1516 = 15/16 sync word bits detected. sync_1616 = 16/16 sync word bits detected (default). sync_3032 = 30/32 sync word bits detected (sync word sent twice). 
                                                            // sync_NONECS = No preamble/sync, carrier-sense above threshold. sync_1516CS = 15/16 + carrier-sense above threshold. sync_1516CS = 15/16 + carrier-sense above threshold. sync_1616CS = 16/16 + carrier-sense above threshold. sync_3032CS = 30/32 (sync word sent twice). + carrier-sense above threshold.
  Smartcc1101.setPRE(SmartCC1101::pre_4);                   // Sets the minimum number of preamble bytes to be transmitted. Possible values: pre_2 : 2, pre_3 : 3, pre_4 : 4 (default), pre_6 : 6, pre_8 : 8, pre_12 : 12, pre_16 : 16, pre_24 : 24
  Smartcc1101.setPQT(0);                                    // Preamble quality estimator threshold. 0 is the derfault. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
  Smartcc1101.setAdrChk(SmartCC1101::adc_NO);               // Controls address check configuration of received packages. adc_NO = No address check (default). adc_YESNOBC = Address check, no broadcast. adc_YES0BC = Address check and 0 (0x00) broadcast. adc_YES0FFBC = Address check and 0 (0x00) and 255 (0xFF) broadcast.
  Smartcc1101.setAddr(0);                                   // Address used for packet filtration. 0 is default. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
  Smartcc1101.setManchester(true);                          // Enables Manchester encoding/decoding. false = Disable (default), true = Enable.
  Smartcc1101.setWhiteData(true);                           // Turn data whitening on / off. false = Whitening off (default). true = Whitening on.
  Smartcc1101.setPktFormat(SmartCC1101::pktf_NORMAL);       // Format of RX and TX data. pktf_NORMAL = Normal mode, use FIFOs for RX and TX (default). pktf_RANDOMTX = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX.
  Smartcc1101.setLengthConfig(SmartCC1101::pktl_VARIABLE);  // pktl_FIXED = Fixed packet length mode. pktl_VARIABLE = Variable packet length mode (default).
  Smartcc1101.setPacketLength(61);                          // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed. 61 is maximum allowed value in this implementation
  Smartcc1101.setFEC(false);                                // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. false = Disable (default), true = Enable.
  Smartcc1101.setCRCCheck(true);                            // true = CRC calculation in TX and CRC check in RX enabled. false = CRC disabled for TX and RX (default).
  Smartcc1101.setCRC_AF(false);                             // Enable automatic flush of RX FIFO when CRC is not OK. (Default: false)
}


void loop() {
  uint8_t buffer[61]{ 0 };  // buffer for the data received by CC1101

  int len = Smartcc1101.receiveData(buffer);
  // len will be 0 if nothing is yet received.
  if (len > 0) {
    Serial.print(len);
    Serial.println(" bytes received.");

    // check transfer quality parameters
    int8_t RSSI = Smartcc1101.getRSSI();
    bool CRC = Smartcc1101.checkCRC();
    uint8_t LQI = Smartcc1101.getLQI();

    // will try to interpret the buffer received as character array, just make sure it's sero-terminated.
    // if it was actually sent from a character arry, this is not necessary
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

## Funtion reference

`bool getCC1101(void)` checks for the presence of a CC1101 module on the SPI bus.

`void init(void)` must be called prijor to using any other funtion. Will establish the necessary setup of the CC1101 chip. Onyl exception: `gtCC1101()`

`void sleep(void)` Sets the CC1101 to sleep mode, reducing power consumption. The next access will wake the chip up again, no special procedure is needed.

 `void setDCFilterOff(bool dcf)` Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud. The recommended IF frequency changes when the DC blocking is disabled. true = Disable (current optimized), false = Enable (better sensitivity).

  `void setCarrierFrequency(uint32_t freq)` Set tranmsision frequency in Hz. Default = 868.35 MHz). The cc1101 can use 300-348 MHZ, 387-464MHZ and 779-928MHZ. More info in the datasheet. Must match sender and receiver

  `void setRXBandWitdth(rx_BandWidth bw)` Set the Receive Bandwidth. Value can be cosen from predefined list of values:
| Bandwidth | Value   |
|----------|---------|
| 58KHz | bw_58kHz |
| 68kHz | bw_68kHz |
| 81kHz | bw_81kHz |
| 102kHz | bw_102kHz |
| 116kHz | bw_116kHz |
| 135kHz | bw_135kHz |
| 203kHz | bw_203kHz |
| 232kHz | bw_232kHz |
| 270kHz | bw_270kHz |
| 325kHz | bw_325kHz |
| 406kHz | bw_406kHz |
| 464kHz | bw_464kHz |
| 541kHz | bw_541kHz |
| 650kHz | bw_650kHz |
| 812kHz | bw_812kHz |


  `void setModulation(Modulation m)` Set modulation mode. Should match sender and receiver. I nearly exclusively use 2FSK modulation.
| Value | Modulation |
|------------|-------|
|mod_2FSK| 2FSK modulation|
|mod_GFSK| GFSK modulation|
|mod_ASKOOK| ASK/OOK modulation (untested)|
|mod_4FSK| 4FSK modulation|
|mod_MSK| MFSK modulation|

  `void setPA(int8_t pa)` Set TXPower in dB. The following settings are possible depending on the frequency band: -30,  -20,  -15,  -10,  -6,    0,    5,    7,    10,   11,   12. Default is max.


  void setDeviation(uint32_t deviation);
  enum sync_Mode : uint8_t {
    sync_NONE = 0b000,    // No preamble/sync
    sync_1516 = 0b001,    // 15/16 sync word bits detected
    sync_1616 = 0b010,    // 16/16 sync word bits detected
    sync_3032 = 0b011,    // 30/32 sync word bits detected
    sync_NONECS = 0b100,  // No preamble/sync, carrier-sense above threshold
    sync_1516CS = 0b101,  // 15/16 + carrier-sense above threshold
    sync_1616CS = 0b110,  // 16/16 + carrier-sense above threshold
    sync_3032CS = 0b111   // 30/32 + carrier-sense above threshold
  };
  void setSyncMode(sync_Mode syncm);
  void setSyncWord(uint8_t sh, uint8_t sl);
  enum preamble_Bytes : uint8_t {
    pre_2 = 0b00000000,   // 2 preamble bytes
    pre_3 = 0b00010000,   // 3 preamble bytes
    pre_4 = 0b00100000,   // 4 preamble bytes
    pre_6 = 0b00110000,   // 6 preamble bytes
    pre_8 = 0b01000000,   // 8 preamble bytes
    pre_12 = 0b01010000,  // 12 preamble bytes
    pre_16 = 0b01100000,  // 16 preamble bytes
    pre_24 = 0b01110000   // 24 preamble bytes
  };
  void setPRE(preamble_Bytes pre);
  void setPQT(uint8_t pqt);
  enum AddressCheck : uint8_t {
    adc_NO,       // No address check
    adc_YESNOBC,  // Address check, no broadcast
    adc_YES0BC,   // Address check and 0 (0x00) broadcast
    adc_YES0FFBC  // Address check and 0 (0x00) and 255 (0xFF) broadcast
  };
  void setAdrChk(AddressCheck adc);
  void setAddr(uint8_t addr);
  enum PacketFormat : uint8_t {
    /*
* @note Synchronous serial mode and Asynchronous serial mode are unsupported
*/
    pktf_NORMAL = 0b00000000,   // Normal mode, use FIFOs for RX and TX
    pktf_RANDOMTX = 0b00100000  // sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
  };
  void setPktFormat(PacketFormat pktf);
  void setCRCCheck(bool crcc);
  void setCRC_AF(bool af);
  void setFEC(bool fec);
  enum PacketLengthConfig : uint8_t {
    /**
* @note CC1101 also supports infinite packet lenth mode, but this is not implemented.
*/
    pktl_FIXED,
    pktl_VARIABLE

  };
  void setLengthConfig(PacketLengthConfig pktl);
  void setPacketLength(uint8_t len);
  void setWhiteData(bool white);
  void setManchester(bool menc);
  void setSymbolRate(double symbolRate);

  void sendData(const char *txBuffer);
  void sendData(const uint8_t *txBuffer, uint8_t size);

  int8_t getRSSI(void);
  bool checkCRC(void);
  uint8_t getLQI(void);
  void setRX(void);
  uint8_t receiveData(uint8_t *rxBuffer);


  void setDelayFunction(void delayFunc(uint8_t));
  void smartDelay(uint8_t ms);

