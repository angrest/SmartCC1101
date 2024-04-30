/**
* @brief CC1101 module library
* @file SmartCC1101.cpp
* @author Axel Grewe
* 
* This library is designed to use CC1101/CC1100 module on the Arduino platform.
* Some care was taken to reduce memory footprint to allow it to run on e.g. an Arduino nano pro.
* Using the functions of the library, you can easily send and receive data by the CC1101/CC1100 module.
* 
* For the details, please refer to the datasheet of CC1100/CC1101.
*
* @note GDO based communication is not implemented.
* @note The number of bytes to send/receive in a single transmission is limited to 61 bytes.
*/


#include "SmartCC1101.h"


/**
* Constants for PA levels - refer to CC1101 data sheet
*/
uint8_t const PROGMEM PA_TABLE_315[8]{ 0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2 };
uint8_t const PROGMEM PA_TABLE_433[8]{ 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0 };
uint8_t const PROGMEM PA_TABLE_868[10]{ 0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0 };
uint8_t const PROGMEM PA_TABLE_915[10]{ 0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0 };

/**
* General helpers
*/

/**
* define custom delay function (optional).
* @param Pointer of function to be called when a delay is needed.
* @return none.
*/
void SmartCC1101::setDelayFunction(delayfunction delayF) {

  delayFunc = delayF;

}

/**
* Call custom delay function set by setDelayFunction.
* @param none
* @return none
*/
void SmartCC1101::smartDelay(uint8_t ms) {
  if(delayFunc)
    delayFunc(ms);
  else
    delay(ms);
}

/**
* SPI helper functions
*/

/**
* Wait CIPO pin going low.
* @param none.
* @return none.
*/
void SmartCC1101::waitCIPO(void) {

  while (digitalRead(CIPO_PIN) > 0) {
	SDEBUGln("CIPO high");  
  }
}

/**
* Pull CSN low, selecting the peripheral and lock bus.
* @param none
* @return none
*/
void SmartCC1101::chipSelect(void) {
#ifdef SPI_HAS_TRANSACTION
  SPI.beginTransaction(mySPISettings);
#endif
  digitalWrite(CS_PIN, LOW);
}

/**
* Pull CSN high, deselecting the peripheral and free bus.
* @param none
* @return none
*/
void SmartCC1101::chipDeselect(void) {
  digitalWrite(CS_PIN, HIGH);
#ifdef SPI_HAS_TRANSACTION
  SPI.endTransaction();
#endif
}

/**
* SPI read/write
*/

/**
* Read single byte value from register of CC1101.
* @param register address to read from.
* @return register value.
*/
uint8_t SmartCC1101::readRegister(uint8_t addr) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_SINGLE);
  uint8_t value = SPI.transfer(0);
  chipDeselect();
  return value;
}

/**
* Read specified number of bytes from CC1101 register in burst mode.
* @param addr Register address.
* @param[out] buffer Array to store register value.
* @param num Number of bytes to read.
* @return none
*/
void SmartCC1101::readBurstRegister(uint8_t addr, uint8_t *buffer, uint8_t num) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_BURST);
  for (uint8_t i = 0; i < num; i++) {
    buffer[i] = SPI.transfer(0);
  }
  chipDeselect();
}

/**
* Read CC1101 status register
* @param addr register address
* @return status value
*/
uint8_t SmartCC1101::readStatusRegister(uint8_t addr) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_BURST);
  uint8_t value = SPI.transfer(0);
  chipDeselect();
  return value;
}

/**
* Write data to CC1101 register.
* @param addr register address.
* @param value register value.
* @return none
*/
void SmartCC1101::writeRegister(uint8_t addr, uint8_t value, [[maybe_unused]] const char *str) {

#ifdef DEBUG_CC1101
  if (addr == 0) {
    SDEBUGln("Ouch: asked to write register address 0 from function %s", str);
  }
#endif

  chipSelect();
  waitCIPO();
  SPI.transfer(addr);
  SPI.transfer(value);
  chipDeselect();
}

/**
* Write specified number of bytes to CC1101 register in burst mode.
* @param addr Register address.
* @param buffer Array of values to write. 
* @param num Number to write.
* @return none
*/
void SmartCC1101::writeBurstRegister(uint8_t addr, const uint8_t *buffer, const uint8_t num, [[maybe_unused]] const char *str) {

#ifdef DEBUG_CC1101
  if (addr == 0) {
    SDEBUGln("Ouch: asked to write register address 0 from function %s", str);
  }
#endif

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | WRITE_BURST);
  for (uint8_t i = 0; i < num; i++) {
    SPI.transfer(buffer[i]);
  }
  chipDeselect();
}

/**
* CC1101 Strobe
* @param strobe Command word
* @return none
*/
uint8_t SmartCC1101::strobe(uint8_t strobe) {

  chipSelect();
  waitCIPO();
  uint8_t value = SPI.transfer(strobe);
  chipDeselect();
  return value;
}

//
// CC1101 basic configuration funtions
//

/**
* CC1101 initialization
* @param none
* @return none
*/
void SmartCC1101::init(void) {

#ifdef SPI_HAS_TRANSACTION
  // works with SPI_MODE0, SPI_MODE2, SPI_MODE3
  mySPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
#else
#warn SPI Transactions are not supported on this board
#endif

  pinMode(SCK_PIN, OUTPUT);
  digitalWrite(SCK_PIN, HIGH);

  pinMode(COPI_PIN, OUTPUT);
  digitalWrite(COPI_PIN, LOW);

  pinMode(CIPO_PIN, INPUT);

  pinMode(CS_PIN, OUTPUT);
  chipDeselect();

#ifdef ESP32
  SPI.begin(SCK_PIN, CIPO_PIN, COPI_PIN, CS_PIN);
#else
  SPI.begin();
#endif

  reset(); //reset first before going further

  configCC1101();  //CC1101 default configuration
}

/**
* CC1101 reset - for details refer to datasheet of CC1101
* @param none
* @return none
*/
void SmartCC1101::reset(void) {
  chipSelect();
  smartDelay(1);
  chipDeselect();
  smartDelay(1);
  chipSelect();
  waitCIPO();

  SPI.transfer(CC1101_SRES);

  waitCIPO();
  chipDeselect();
}

/**
* Set CC1101 to IDLE state.
* @param none
* @return none
*/
void SmartCC1101::setIDLEState(void) {

  if (getState() == state_IDLE)  // return immediately if already idle
    return;
  strobe(CC1101_SIDLE);
  while (getState() != state_IDLE) {
	SDEBUGln("Waiting for IDLE");
  }
  
    ;  // wait until state is IDLE
}

/**
* Set CC1101 in sleep mode.
* @param none
* @return none
*/
void SmartCC1101::sleep(void) {

  sleepState=true;
  setIDLEState();       // Exit RX / TX, turn off frequency synthesizer and exit
  strobe(CC1101_SPWD);  // Enter power down mode when CSn goes high.
}

/**
* return the state of the chip (refer SWRS061I page 31)
* @param none
* @return State of CC1101
* Possible return values:
* state_IDLE			= 0b000
* state_RX				= 0b001
* state_TX				= 0b010
* state_FSTXON			= 0b011
* state_CALIBRATE		= 0b100
* state_SETTLING		= 0b101
* state_RXFIFO_OVERFLOW = 0b110
* state_TXFIFO_UNDERFLOW = 0b111
*/
SmartCC1101::chipState SmartCC1101::getState(void) {

  uint8_t old_state = strobe(CC1101_SNOP);
  // we read until same result is read twice
  while (1) {
    uint8_t state = strobe(CC1101_SNOP);
    if (state == old_state)
      break;
	SDEBUGln("State readout mismatch, try again");
    old_state = state;
  }
  chipState rc = static_cast<chipState>((old_state >> 4) & 0b00111);
  return rc;
}

/**
* CC1101 basic register config, refer to e.g. Smart RF Studio
* @param none
* @return none
*/
void SmartCC1101::configCC1101(void) {

  reset();

  writeRegister(CC1101_PKTLEN, 0x3d);          // 61 Bytes - max possible TX length
  writeRegister(CC1101_PKTCTRL1, 0b00001100);  // PQT=0, append status, FIFO autoflush on CRC error
  writeRegister(CC1101_MDMCFG4, 0b00000000);   // Max Channel BW 812.5 Khz
  writeRegister(CC1101_MDMCFG2, 0b00001010);   // DC Filter enable, 2-FSK, Manch Enc, 16/16 Sync words detected
  writeRegister(CC1101_MDMCFG1, 0b00000010);   // 2 Preamble bytes, exponent 2 in channel spacing
  writeRegister(CC1101_MCSM0, 0b00011000);     // Autocal when IDLE to RX, PO_TIMEOUT=2

  // Optized for sensitivty @835.35 Mhz, 100kb symbol rate
  writeRegister(CC1101_FSCTRL1, 0x08);  //Frequency Synthesizer Control
  writeRegister(CC1101_FSCTRL0, 0x00);  //Frequency Synthesizer Control

  writeRegister(CC1101_FOCCFG, 0x1D);    //Frequency Offset Compensation Configuration
  writeRegister(CC1101_BSCFG, 0x1C);     //Bit Synchronization Configuration
  writeRegister(CC1101_AGCCTRL2, 0xC7);  //AGC Control
  writeRegister(CC1101_AGCCTRL1, 0x00);  //AGC Control
  writeRegister(CC1101_AGCCTRL0, 0xB2);  //AGC Control
  writeRegister(CC1101_WORCTRL, 0xFB);   //Wake On Radio Control
  writeRegister(CC1101_FREND1, 0xB6);    //Front End RX Configuration
  writeRegister(CC1101_FREND0, 0x10);    //Front End TX Configuration
  writeRegister(CC1101_FSCAL3, 0xEA);    //Frequency Synthesizer Calibration
  writeRegister(CC1101_FSCAL2, 0x2A);    //Frequency Synthesizer Calibration
  writeRegister(CC1101_FSCAL1, 0x00);    //Frequency Synthesizer Calibration
  writeRegister(CC1101_FSCAL0, 0x1F);    //Frequency Synthesizer Calibration
  writeRegister(CC1101_TEST0, 0x09);     //Various Test Settings

  // the easiest way to have settings managed cosnsistently and onyly change in one place
  onWakeup();

  setCarrierFrequency(cfreq);
}


/**
* Rewrite settings of the registers which are reset on sleep
* @param none
* @return none
*/
void SmartCC1101::onWakeup(void) {

  SDEBUGln("Wakeup from sleep");
  sleepState=false;
  writeRegister(CC1101_AGCTEST, 0x3F);   //AGC Test
  writeRegister(CC1101_TEST0, 0x09);     //Various Test Settings
  // index 0 is preserved in PATABLE, ASK/OOK use index 1
  if (modulation == mod_ASKOOK)
    setPA(pa);

}


/**
* Test SPI connection to CC1101 and return true if a value is read back from the status register CC1101_VERSION.
* @param none
* @return true/false
*/
bool SmartCC1101::getCC1101(void) {

  uint8_t version = readStatusRegister(CC1101_VERSION);
/**
* @note All chips at hand return 4, but this is bound to change
* so, we accept all values which are nonzero and not 0xFF,
* the latter is returned when the connection is broken (e.g. chip not connected).
*/
  if ((version > 0) && (version < 0xFF)) {
    return true;
  } else {
    return false;
  }
}

/**
* Configuration functions for user
*/

/**
* Disable digital DC blocking filter before demodulator
* @param filter true/false
* @return none
*/
void SmartCC1101::setDCFilterOff(bool dcf) {

  uint8_t state = readRegister(CC1101_MDMCFG2);

  if (dcf == true)
    state |= 0b10000000;
  else
    state &= 0b01111111;

  writeRegister(CC1101_MDMCFG2, state);
}

/**
* Set Receive bandwidth in kHz
* @param Bandwidth
* Possible values:
* bw_58kHz  = 0b11110000,
* bw_68kHz  = 0b11100000,
* bw_81kHz  = 0b11010000,
* bw_102kHz = 0b11000000,
* bw_116kHz = 0b10110000,
* bw_135kHz = 0b10100000,
* bw_162kHz = 0b10010000,
* bw_203kHz = 0b10000000,
* bw_232kHz = 0b01110000,
* bw_270kHz = 0b01100000,
* bw_325kHz = 0b01010000,
* bw_406kHz = 0b01000000,
* bw_464kHz = 0b00110000,
* bw_541kHz = 0b00100000,
* bw_650kHz = 0b00010000,
* bw_812kHz = 0b00000000
* @return none
*/
void SmartCC1101::setRXBandWitdth(rx_BandWidth bw) {

  setIDLEState();
  uint8_t state = readRegister(CC1101_MDMCFG4);
  state &= 0b00001111;

  writeRegister(CC1101_MDMCFG4, state | bw);
}

/**
* Set CC1101 PA Power. Not persisted during sleep, except index 0
* For ASK/OOK, PA needs to be set again after sleep
* @param PA Power in dB
* @return none
*/
void SmartCC1101::setPA(int8_t p) {
  uint8_t PA_TABLE[8]{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  int8_t a = 0;
  uint8_t index=0;
  pa = p;

  if (cfreq >= 300000000 && cfreq <= 348000000) {
    if (pa <= -30) {
	    index=0;
    } else if (pa > -30 && pa <= -20) {
		index=1;
    } else if (pa > -20 && pa <= -15) {
		index=2;
    } else if (pa > -15 && pa <= -10) {
      index=3;
    } else if (pa > -10 && pa <= 0) {
      index=4;
    } else if (pa > 0 && pa <= 5) {
      index=5;
    } else if (pa > 5 && pa <= 7) {
      index=6;
    } else if (pa > 7) {
      index=7;
    }
    a = pgm_read_byte(&PA_TABLE_315[index]);
  } else if (cfreq >= 378000000 && cfreq <= 464000000) {
    if (pa <= -30) {
	    index=0;
    } else if (pa > -30 && pa <= -20) {
	    index=1;
    } else if (pa > -20 && pa <= -15) {
	    index=2;
    } else if (pa > -15 && pa <= -10) {
	    index=3;
    } else if (pa > -10 && pa <= 0) {
	    index=4;
    } else if (pa > 0 && pa <= 5) {
	    index=5;
    } else if (pa > 5 && pa <= 7) {
	    index=6;
    } else if (pa > 7) {
	    index=7;
    }
    a = pgm_read_byte(&PA_TABLE_433[index]);
  } else if (cfreq >= 779000000 && cfreq < 900000000) {
    if (pa <= -30) {
	    index=0;
    } else if (pa > -30 && pa <= -20) {
	    index=1;
    } else if (pa > -20 && pa <= -15) {
	    index=2;
    } else if (pa > -15 && pa <= -10) {
	    index=3;
    } else if (pa > -10 && pa <= -6) {
	    index=4;
    } else if (pa > -6 && pa <= 0) {
	    index=5;
    } else if (pa > 0 && pa <= 5) {
	    index=6;
    } else if (pa > 5 && pa <= 7) {
	    index=7;
    } else if (pa > 7 && pa <= 10) {
	    index=8;
    } else if (pa > 10) {
 	    index=9;
    }
    a = pgm_read_byte(&PA_TABLE_868[index]);
  } else if (cfreq >= 900000000 && cfreq <= 928000000) {
    if (pa <= -30) {
	    index=0;
    } else if (pa > -30 && pa <= -20) {
	    index=1;
    } else if (pa > -20 && pa <= -15) {
	    index=2;
    } else if (pa > -15 && pa <= -10) {
	    index=3;
    } else if (pa > -10 && pa <= -6) {
	    index=4;
    } else if (pa > -6 && pa <= 0) {
	    index=5;
    } else if (pa > 0 && pa <= 5) {
	    index=6;
    } else if (pa > 5 && pa <= 7) {
	    index=7;
    } else if (pa > 7 && pa <= 10) {
	    index=8;
    } else if (pa > 10) {
	    index=9;
    }
    a = pgm_read_byte(&PA_TABLE_915[index]);
  }
  
  if (modulation == mod_ASKOOK) {
    PA_TABLE[1] = a;  // PA power for sending a '1'
  } else {
	SDEBUGln("PA = %0x",a);
    PA_TABLE[0] = a;  // PA Power
  }
  writeBurstRegister(CC1101_PATABLE, PA_TABLE, 8);
}

/**
* Set the carrier frequency.
* @param Frequency in Hz
* @return none
*/
void SmartCC1101::setCarrierFrequency(uint32_t frequency) {
  // Calculate the frequency register value using 64-bit arithmetic to prevent overflow
  uint32_t frequencyRegisterValue = ((uint64_t)frequency << 16) / CC1101_CRYSTAL_FREQUENCY;

  // Extract the individual bytes for the frequency registers
  uint8_t frequencyHigh = (frequencyRegisterValue >> 16) & 0xFF; // High byte (FREQ2)
  uint8_t frequencyMid = (frequencyRegisterValue >> 8) & 0xFF;   // Middle byte (FREQ1)
  uint8_t frequencyLow = frequencyRegisterValue & 0xFF;          // Low byte (FREQ0)

  // Set the transceiver to IDLE state before writing the frequency registers
  setIDLEState();

  writeRegister(CC1101_FREQ2, frequencyHigh);
  writeRegister(CC1101_FREQ1, frequencyMid);
  writeRegister(CC1101_FREQ0, frequencyLow);
  
  cfreq = frequency;
}


/**
* set CC1101 Modulation
* @param Modulation
* Possible values:
* mod_2FSK
* mod_GFSK
* mod_ASKOOK
* mod_4FSK
* mod_MSK
* @return none
*/
void SmartCC1101::setModulation(Modulation mod) {

  uint8_t frend0 = 0x10;  // per default, use index 0 for PA setting

  if (mod == mod_ASKOOK)
    frend0 = 0x11;  // use PATABLE index 1 for transmitting '1'

  modulation = mod;

  uint8_t state = readRegister(CC1101_MDMCFG2);
  SDEBUGln("CC1101_MDMCFG2 before %0x",state);
  state &= 0b10001111;
  SDEBUGln("CC1101_MDMCFG2 after %0x",state| mod);

  writeRegister(CC1101_MDMCFG2, state | mod);
  writeRegister(CC1101_FREND0, frend0);
  setPA(pa);
}

/**
* Transmission configuration
*/

/**
* Set PKT_FORMAT
* Format of RX and TX data (this should be "normal" in most cases)
* @param Packet format
* pktf_NORMAL    Normal mode, use FIFOs for RX and TX
* pktf_RANDOMTX  Sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
* @return none
*
* @note Synchronous serial mode and Asynchronous serial mode are unsupported
*/
void SmartCC1101::setPktFormat(PacketFormat pktf) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);
  state &= 0b11001111;

  writeRegister(CC1101_PKTCTRL0, state | pktf);
}

/**
* Controls address check configuration of received packages
* @param Address Check configtuation
* Possible values:
* adc_NO			No address check
* adc_YESNOBC		Address check, no broadcast
* adc_YES0BC		Address check and 0 (0x00) broadcast
* adc_YES0FFBC	Address check and 0 (0x00) and 255 (0xFF) broadcast
* @return none
*/
void SmartCC1101::setAdrChk(AddressCheck adc) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);
  state &= 0b11111100;

  writeRegister(CC1101_PKTCTRL1, state | adc);
}

/**
* Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
* @param Channel number
* @return none
*/
void SmartCC1101::setAddr(uint8_t addr) {
  writeRegister(CC1101_ADDR, addr);
}

/**
* CRC calculation in TX and CRC check in RX
* @param Boolean flag if CRC should be checked
* @return none
*/
void SmartCC1101::setCRCCheck(bool crc) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);

  if (crc == true)
    state |= 0b00000100;
  else
    state &= 0b11111011;

  writeRegister(CC1101_PKTCTRL0, state);
}

/**
* Enable automatic flush of RX FIFO when CRC is not OK
* (requires CRC checking enabled by calling setCRCCheck(true) first
* @param Boolean flag to enabel autoflush
* @return none
*/
void SmartCC1101::setCRC_AF(bool af) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);

  if (af == true)
    state |= 0b00001000;
  else
    state &= 0b11110111;

  writeRegister(CC1101_PKTCTRL1, state);
}

/**
* Enable Forward Error Correction (FEC). Works only in fxed length mode
* @param Boolean flag to enable FEC
* @return none
*/
void SmartCC1101::setFEC(bool fec) {

  uint8_t state = readRegister(CC1101_MDMCFG1);
  if (fec == true)
    state |= 0b10000000;
  else
    state &= 0b01111111;

  writeRegister(CC1101_MDMCFG1, state);
}


/**
* Configure the packet length
* @param Length control mode
* Possible values:
* pktl_FIXED Fixed packet length mode. Length configured with setPacketLength.
* pktl_VARIABLE Variable packet length mode. Length configured with setPacketLength indicates maximum allowed packet length.
* Theoretically, the CC1101 also supports infinite packet length mode, but this is not implemented.
* @return none
*/
void SmartCC1101::setLengthConfig(PacketLengthConfig lenc) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);
  state &= 0b11111100;

  writeRegister(CC1101_PKTCTRL0, state | lenc);
}

/**
* Sets the packet length in fixed length mode.
* If variable packet length mode is used, this value indicates the maximum packet length allowed.
* @param Packet Length
* @return none
*/
void SmartCC1101::setPacketLength(uint8_t pktlen) {
/**
* @note although 255 bytes are theoretically possible, the code here is limited
* to using a single RXBUFFER/TXBUFFER with the length of 64 bytes, reuslting in net 61 bytes of payload
*/
  if (pktlen > 61)
    pktlen = 61;
  writeRegister(CC1101_PKTLEN, pktlen);
}

/**
* Set Sync Word
* @param Syncword high, low
* Allthough changing this is possible, the datasheet does not recommend it. Must match in sender/receiver.
* @return none
*/
void SmartCC1101::setSyncWord(uint8_t sh, uint8_t sl) {
  writeRegister(CC1101_SYNC1, sh);
  writeRegister(CC1101_SYNC0, sl);
}

/**
* Combined sync-word qualifier mode. Must match in sender/receiver.
* @param Sync Mode
* Possible values:
* sync_NONE   No preamble/sync
* sync_1516   15/16 sync word bits detected
* sync_1616   16/16 sync word bits detected
* sync_3032   30/32 sync word bits detected
* sync_NONECS No preamble/sync, carrier-sense above threshold
* sync_1516CS 15/16 + carrier-sense above threshold
* sync_1616CS 16/16 + carrier-sense above threshold
* sync_3032CS 30/32 + carrier-sense above threshold
* @return none
*/
void SmartCC1101::setSyncMode(sync_Mode syncm) {

  uint8_t state = readRegister(CC1101_MDMCFG2);
  SDEBUGln("CC1101_MDMCFG2 before %0x",state);
  state &= 0b11111000;
  SDEBUGln("CC1101_MDMCFG2 after %0x",state | syncm);

  writeRegister(CC1101_MDMCFG2, state | syncm);
}

/**
* Sets the minimum number of preamble bytes to be transmitted. 
* @param Preamble Bytes
* Possible values:
* pre_2
* pre_3
* pre_4
* pre_6
* pre_8
* pre_12
* pre_16
* pre_24
* @return none
*/
void SmartCC1101::setPRE(preamble_Bytes pre) {

  uint8_t state = readRegister(CC1101_MDMCFG1);
  state &= 0b10001111;

  writeRegister(CC1101_MDMCFG1, state | pre);
}

/**
* Preamble quality estimator threshold.
* A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
* @param PQT, range from 0-7
* @return none
*/
void SmartCC1101::setPQT(uint8_t pqt) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);
  state &= 0b00011111;

  pqt <<= 5; // this takes care we do not use alues exceeding range

  writeRegister(CC1101_PKTCTRL1, state | pqt);
}

/**
* Enables Manchester encoding/decoding. Must match in sender/receiver.
* @param none
* @return none
*/
void SmartCC1101::setManchester(bool menc) {

  uint8_t state = readRegister(CC1101_MDMCFG2);
  if (menc == true)
    state |= 0b00001000;
  else
    state &= 0b11110111;

  writeRegister(CC1101_MDMCFG2, state);
}

/**
* Turn data whitening on / off. Must match in sender/receiver.
* @param none
* @return none
*/
void SmartCC1101::setWhiteData(bool white) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);

  if (white == true)
    state |= 0b01000000;
  else
    state &= 0b10111111;

  writeRegister(CC1101_PKTCTRL0, state);
}

/**
* Set RX/TX symbolrate. Must match in sender/receiver.
* @param Symbol rate in bit/s
* @return none
* @note The implementation uses floating point calculations as the rounding errors are too big for caltculating the mantissa using int-math
*/
void SmartCC1101::setSymbolRate(double symbolRate) {

  symbolRate = constrain(symbolRate, CC1101_CRYSTAL_FREQUENCY*1./pow(2,28), CC1101_CRYSTAL_FREQUENCY*511./pow(2,13));
  double symR = symbolRate;

  uint8_t drate_e = 0;
  
  // Find the exponent (drate_e)
  const double minf = 511.*CC1101_CRYSTAL_FREQUENCY/pow(2,28); // drate_m is 255 max

  while (symR > minf) { 
    drate_e++;         // Increment exponent
    symR /=2. ;        // Divide factor by 2
  }

  // ... and calculate the mantissa value
  double drate_mf = symbolRate*pow(2,28-drate_e)/CC1101_CRYSTAL_FREQUENCY;
  uint8_t drate_m = (drate_mf+.5) - 256;

  setIDLEState(); // Ensure CC1101 is in IDLE state before configuring
  // Read the current value of MDMCFG4 and mask out DRATE bits
  uint8_t currentMdmCfg4 = readRegister(CC1101_MDMCFG4) & 0b11110000;

  // Write the exponent and mantissa to the CC1101 registers
  writeRegister(CC1101_MDMCFG4, currentMdmCfg4 | drate_e);
  writeRegister(CC1101_MDMCFG3, drate_m);

}

/**
* Modem Deviation Setting. Must match on sender/receiver.
* @li for 2-FSK, GFSK, 4-FSK Specifies the nominal frequency deviation from the carrier for a ‘0’ (-DEVIATN) and ‘1’ (+DEVIATN). 
* @li MSK Specifies the fraction of symbol period (1/8-8/8) during which a phase change occurs (‘0’: +90deg, ‘1’:-90deg).
* @param Deviation in Hz
* @return none
* @note the algorithm rounds down, so it will set the parameters always to the lower value of the interval
*/
void SmartCC1101::setDeviation(uint32_t deviation) {

  // Constrain the deviation value within the acceptable range
  deviation = constrain(deviation, CC1101_CRYSTAL_FREQUENCY >> 14, (CC1101_CRYSTAL_FREQUENCY >> 10) * 15);
  uint32_t devtn = deviation;
  
  uint8_t deviation_e = 0;
  
  // Find the exponent (deviation_e)
  while (devtn >= (CC1101_CRYSTAL_FREQUENCY >> 13)) { // Mantissa is 15 max
    deviation_e++;      // Increment exponent
    devtn >>= 1;        // Divide factor by 2
  }
  
  // ... and calculate the mantissa value. Rounding errors might lead to a one-off result on the exact interval limit.
  int32_t deviation_m = (deviation << (17-deviation_e)) / CC1101_CRYSTAL_FREQUENCY - 8;
  
  SDEBUGln("deviation=%lu, deviation_m=%d, devation_e=%u",deviation,deviation_m,deviation_e);
  
  // bad things could happen due to rounding issues, just to be sure 
  deviation_m = constrain(deviation_m,0,7);

  // Write the calculated values to the CC1101_DEVIATN register
  writeRegister(CC1101_DEVIATN, (deviation_e << 4) | deviation_m);
}


/**
* Send data
* @param txBuffer zero-terminated character array to send, no more than 61
* @return none
*/
void SmartCC1101::sendData(const char *txBuffer) {
  sendData((uint8_t *)txBuffer, (uint8_t)strlen(txBuffer));
}

/**
* Send data
* @param txBuffer Data array to send
* @param size Number of data to send, no more than 61
* @return none
*/
void SmartCC1101::sendData(const uint8_t *txBuffer, uint8_t size) {

  uint8_t state;
 
  if(sleepState)
    onWakeup();
 
  SDEBUGln("Got %d bytes to write: %s", size, txBuffer);

  // limit to 61 characters
  size = (size <= 61) ? size : 61;

  setIDLEState();

  writeRegister(CC1101_TXFIFO, size);
  
  writeBurstRegister(CC1101_TXFIFO, txBuffer, size);  //write data to send
  strobe(CC1101_STX);                                 //start send

  // Source Datasheet p 58, ch 22.1
  state = getState();

  SDEBUGln("State = %d",state);
  while ((state == state_CALIBRATE) || (state == state_SETTLING)) {
    SDEBUGln("Calibrating");
    state = getState();
  }


  while (1) {
    state = getState();
    SDEBUGln("State = %d", state);
    if (state == state_IDLE) break;  // we wait for IDLE state
    
    smartDelay(2);

  }
  SDEBUGln("Send took %ld ms", millis()-startTime);
  
  strobe(CC1101_SFTX);  //flush TXfifo
}

/**
* set CC1101 to receive state
* @param none
* @return none
*/
void SmartCC1101::setRX(void) {

  if(sleepState)
    onWakeup();

  // reset values from last receive
  rssi = 0;
  lqi = 0;
  crc = false;

  uint8_t state = getState();

  // wait until calibration or settling finishes
  while ((state == state_CALIBRATE) || (state == state_SETTLING)) {
    state = getState();
  }

  // if already in RX or RX buffer overflow return immediately
  if ((state == state_RX) || (state == state_RXFIFO_OVERFLOW))
    return;

  // Handle TX buffer underflow here directly
  if (state == state_TXFIFO_UNDERFLOW)
    strobe(CC1101_SFTX);

  //N all other states are unexpected, force to IDLE first
  if (state != 0) {
    SDEBUGln("Unexpected state %0x", state);
    setIDLEState();
  }

  SDEBUGln("Before autocal: FSCAL1 = %0x", readRegister(CC1101_FSCAL1));

  strobe(CC1101_SRX);  //start receive (autcal on IDLE->RX

  // Check - Source Datasheet p 58, ch 22.1
  state = getState();
  while ((state == state_CALIBRATE) || (state == state_SETTLING)) {
    SDEBUGln("Calibrating");
    state = getState();
  }
  uint8_t lock = readRegister(CC1101_FSCAL1);
  if (lock != 0x3f)
    SDEBUGln("Calibration locked FSCAL1 = %0x", readRegister(CC1101_FSCAL1));
}

/**
* Get RSSI Level
* read current RSSI Level if not IDLE, the one from last RX otherwise
* @param none
* @return RSSI value in DB (negative value)
*/
int8_t SmartCC1101::getRSSI(void) {
  if (getState() != state_IDLE)
    return (getRSSI(readStatusRegister(CC1101_RSSI)));
  else
    return rssi;
}

/**
* Get RSSI Level
* Calculating the RSSI Level for a given value
* @param raw data readout value from register
* @return RSSI value in DB (negative value)
*/
int8_t SmartCC1101::getRSSI(uint8_t rawValue) {

  int16_t rssi = rawValue;

  if (rssi >= 128) {
    rssi = (rssi - 256) / 2 - 74;
  } else {
    rssi = (rssi / 2) - 74;
  }
  return (int8_t)rssi;
}

/**
* Check CRC
* read current CRC if not IDLE, the one from last RX otherwise
* @param none
* @return true if CRC in RX data is ok (or CRC disabled), false otherwise
*/
bool SmartCC1101::checkCRC(void) {

  if (getState() != state_IDLE)
    return checkCRC(readStatusRegister(CC1101_LQI));
  else
    return crc;
}

/**
* Check CRC
* calculate CRC for a given raw value
* @param Raw data readout value from register
* @return true if CRC in RX data is ok, false otherwise
*/
bool SmartCC1101::checkCRC(uint8_t rawValue) {

  if (rawValue & 0b10000000)
    return true;
  else
    return false;
}

/**
* Link Quality Indication (LQI)
* read current LQI if not IDLE, the one from last RX otherwise
* @param none
* @return LQI value (lower is better)
*/
uint8_t SmartCC1101::getLQI(void) {
  if (getState() != state_IDLE)
    return getLQI(readStatusRegister(CC1101_LQI));
  else
    return lqi;
}

/**
* Link Quality Indication (LQI)
* Calculate LQI for given raw value
* @param raw data readout value from register
* @return LQI value (lower is better)
*/
uint8_t SmartCC1101::getLQI(uint8_t rawValue) {
  return (rawValue & 0b01111111);
}

/**
* read data received from RXfifo
* @param[out] rxBuffer buffer to store data
* @return number of bytes received, 0 if:
* @li still RX'ing
* @li inconsisntent state or 
* @li CRC error (if setCRC_AF(true))
*
*/

uint8_t SmartCC1101::receiveData(uint8_t *rxBuffer) {
	
  if(sleepState)
    onWakeup();

  uint8_t size = 0;
  uint8_t state = getState();
  if (state == state_RX) {  // RX not yet complete, will be idle if RX finshed
    return 0;
  }

  if (state == state_RXFIFO_OVERFLOW)
    SDEBUGln("RX FIFO overflow");
  // This is the total number of bytes in buffer including length byte plus RSSI and LQI
  uint8_t rxbytes = readStatusRegister(CC1101_RXBYTES) & BYTES_IN_RXFIFO;

  if (rxbytes > 0) {
    size = readRegister(CC1101_RXFIFO);  // this returns the payload length
    uint8_t append = readRegister(CC1101_PKTCTRL1) & 0b00000100;

    if ((size > 0) && ((size == (rxbytes - 3)) || state == state_RXFIFO_OVERFLOW)) {
      // payload should be total-3 except in buffer overflow condition, where total==1
      readBurstRegister(CC1101_RXFIFO, rxBuffer, size);
      if (append) {  // next two bytes are status in case appedStatus is set (which it should be)
        uint8_t status[2];
        readBurstRegister(CC1101_RXFIFO, status, 2);
        rssi = getRSSI(status[0]);
        lqi = getLQI(status[1]);
        crc = checkCRC(status[1]);
      }
      uint8_t rem = rxbytes - (size + 3);
      if ((rem > 0) && (state != state_RXFIFO_OVERFLOW)) {
        SDEBUGln("FIFO STILL HAS %d BYTES", rem);
      }
    } else {
      SDEBUGln("Wrong rx size=%d rxbytes=%d", size, rxbytes);
      readBurstRegister(CC1101_RXFIFO, rxBuffer, 64);
      SDEBUGln("%63s", (char *)rxBuffer);
      size = 0;
    }
  } else {
    // if buffer is empty after transition for RX to IDLE, most probably CRC_AF was set and the beffer got cleared on a CRC error
    rssi = 0;
    lqi = 0;
    crc = false;
  }

  // all done, clear RX FIFO
  strobe(CC1101_SFRX);

  return size;
}

#ifdef DEBUG_CC1101
/**
* output register contents to console
*/

void SmartCC1101::dumpRegs(void) {

  SDEBUGln("CC1101 Register Dump:");
  for (int i = 0; i < 0x40; i++) {
    SDEBUGln("Register %0x: %0x", i, readStatusRegister(i));
  }
}
#endif

SmartCC1101 Smartcc1101;
