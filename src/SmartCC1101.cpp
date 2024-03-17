//****************************************************************
//
//  SmartCC1101.cpp - CC1101 module library
//  Copyright (c) 2024 Axel Grewe.
//
//  This library is designed to use CC1101/CC1100 module on the Arduino platform.
//  Some care was taken to reduce memory footprint to allow it to run on e.g. an Arduino nano pro
//  Using the functions of the library, you can easily send and receive data by the CC1101/CC1100 module.
//  
//  For the details, please refer to the datasheet of CC1100/CC1101.
//
//  GDO based communication is not implemented.
//----------------------------------------------------------------------------------------------------------------
//  Inspired by code from ELECHOUSE, LSatan and pkarsy
//****************************************************************


#include "SmartCC1101.h"


// Constants for PA levels - refer to CC1101 data sheet
uint8_t const PROGMEM PA_TABLE_315[8]{ 0x12, 0x0D, 0x1C, 0x34, 0x51, 0x85, 0xCB, 0xC2 };
uint8_t const PROGMEM PA_TABLE_433[8]{ 0x12, 0x0E, 0x1D, 0x34, 0x60, 0x84, 0xC8, 0xC0 };
uint8_t const PROGMEM PA_TABLE_868[10]{ 0x03, 0x17, 0x1D, 0x26, 0x37, 0x50, 0x86, 0xCD, 0xC5, 0xC0 };
uint8_t const PROGMEM PA_TABLE_915[10]{ 0x03, 0x0E, 0x1E, 0x27, 0x38, 0x8E, 0x84, 0xCC, 0xC3, 0xC0 };


//
// SPI helper functions
//

//****************************************************************
// Funtion Name: waitCIPO
// Funtion     : Wait CIPO pin going low
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::waitCIPO() {
  while (digitalRead(CIPO_PIN) > 0)
    ;
}

//****************************************************************
// Funtion Name: chipSelect
// Funtion     : Pull CSN low, selecting the peripheral and lock bus
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::chipSelect() {
#ifdef SPI_HAS_TRANSACTION
  SPI.beginTransaction(mySPISettings);
#endif
  digitalWrite(CS_PIN, LOW);
}

//****************************************************************
// Funtion Name: chipDeselect
// Funtion     : Pull CSN high, deselecting the peripheral and free bus
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::chipDeselect() {
  digitalWrite(CS_PIN, HIGH);
#ifdef SPI_HAS_TRANSACTION
  SPI.endTransaction();
#endif
}

//
// SPI read/write
//

//****************************************************************
// Funtion Name: readRegister
// Funtion     : CC1101 read data from register
// Input       : addr: register address
// Output      : register value
//****************************************************************
uint8_t SmartCC1101::readRegister(uint8_t addr) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_SINGLE);
  uint8_t value = SPI.transfer(0);
  chipDeselect();
  return value;
}

//****************************************************************
// Funtion Name: readBurstRegister
// Funtion     : CC1101 read burst data from register
// Input       : addr: register address; buffer: array to store register value; num: number to read
// Output      : none
//****************************************************************
void SmartCC1101::readBurstRegister(uint8_t addr, uint8_t *buffer, uint8_t num) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_BURST);
  for (uint8_t i = 0; i < num; i++) {
    buffer[i] = SPI.transfer(0);
  }
  chipDeselect();
}

//****************************************************************
// Funtion Name: readStatusRegister
// Funtion     : CC1101 read status register
// Input       : addr: register address
// Output      : status value
//****************************************************************
uint8_t SmartCC1101::readStatusRegister(uint8_t addr) {

  chipSelect();
  waitCIPO();
  SPI.transfer(addr | READ_BURST);
  uint8_t value = SPI.transfer(0);
  chipDeselect();
  return value;
}

//****************************************************************
// Funtion Name: writeRegister
// Funtion     : CC1101 write data to register
// Input       : addr: register address; value: register value
// Output      : none
//****************************************************************
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

//****************************************************************
// Funtion Name: writeBurstRegister
// Funtion     : CC1101 write burst data to register
// Input       : addr: register address; buffer: register value array; num: number to write
// Output      : none
//****************************************************************
void SmartCC1101::writeBurstRegister(uint8_t addr, uint8_t *buffer, uint8_t num, [[maybe_unused]] const char *str) {

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

//****************************************************************
// Funtion Name: strobe
// Funtion     : CC1101 Strobe
// Input       : strobe: command word
// Output      : none
//****************************************************************
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

//****************************************************************
// Funtion Name: init
// Funtion     : CC1101 initialization
// Input       : none
// Output      : none
//****************************************************************
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

//****************************************************************
// Funtion Name: reset
// Funtion     : CC1101 reset - for details refer to datasheet of CC1101
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::reset(void) {
  chipSelect();
  delay(1);
  chipDeselect();
  delay(1);
  chipSelect();
  waitCIPO();

  SPI.transfer(CC1101_SRES);

  waitCIPO();
  chipDeselect();
}

//****************************************************************
// Funtion Name: setIDLEstate
// Funtion     : set Rx / TX Off
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::setIDLEstate() {

  if (getState() == state_IDLE)  // return immediately if already idle
    return;
  strobe(CC1101_SIDLE);
  while (getState() != state_IDLE)
    ;  // wait until state is IDLE
}

//****************************************************************
// Funtion Name: goSleep
// Funtion     : set cc1101 Sleep on
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::sleep(void) {

  sleepState=true;
  setIDLEstate();       // Exit RX / TX, turn off frequency synthesizer and exit
  strobe(CC1101_SPWD);  // Enter power down mode when CSn goes high.
}

//****************************************************************
// Funtion Name: getState
// Funtion     : return the state of the chip SWRS061I page 31
// Input       : none
// Output      : State of CC1101
// state_IDLE			= 0b000
// state_RX				= 0b001
// state_TX				= 0b010
// state_FSTXON			= 0b011
// state_CALIBRATE		= 0b100
// state_SETTLING		= 0b101
// state_RXFIFO_OVERFLOW = 0b110
// state_TXFIFO_UNDERFLOW = 0b111
//****************************************************************
SmartCC1101::chipState SmartCC1101::getState() {

  uint8_t old_state = strobe(CC1101_SNOP);
  // we read until same result is read twice
  while (1) {
    uint8_t state = strobe(CC1101_SNOP);
    if (state == old_state) {

      chipState rc = static_cast<chipState>((state >> 4) & 0b00111);
      return rc;
      //      return (state >> 4) & 0b00111;
    }
    old_state = state;
  }
}

//****************************************************************
// Funtion Name: configCC1101
// Funtion     : CC1101 basic register config refer e.g. Smart RF Studio
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::configCC1101(void) {

  reset();

  writeRegister(CC1101_PKTLEN, 0x3d);          // 61 Bytes - max possible TX length
  writeRegister(CC1101_PKTCTRL1, 0b00001100);  // PQT=0, append status, FIFO autoflush on CRC error
  writeRegister(CC1101_MDMCFG4, 0b00000000);   // Max Channel BW 812.5 Khz
  writeRegister(CC1101_MDMCFG2, 0b00001010);   // DC Filter enable, 2-FSK, Manch Enc, 16/16 Sync words detected
  writeRegister(CC1101_MDMCFG1, 0b00000010);   // 2 Preamble bytes, exponent 2 in channel spacing
  writeRegister(CC1101_MCSM0, 0b00111000);     // Autocal every 4th time when going from RX/TX to IDLE, PO_TIMEOUT=2

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

  setFrequency(MHz * 1e6);
}


//****************************************************************
// Funtion Name: onWakeup
// Funtion     : those registers are reset on sleep
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::onWakeup(void) {

  SDEBUGln("Wakeup from sleep");
  sleepState=false;
  writeRegister(CC1101_AGCTEST, 0x3F);   //AGC Test
  writeRegister(CC1101_TEST0, 0x09);     //Various Test Settings
  // index 0 is preserved in PATABLE, ASK/OOK use index 1
  if (modulation == mod_ASKOOK)
    setPA(pa);

}


//****************************************************************
// Funtion Name: getCC1101
// Funtion     : Test SPI connection and return true when true.
// Input       : none
// Output      : none
//****************************************************************
bool SmartCC1101::getCC1101(void) {

  uint8_t version = readStatusRegister(CC1101_VERSION);
  // all chips at hand return 4, but this is bound to change
  // so, we accept all values which are nonzero and not 0xFF,
  // the latter is returned when the connection is broken (e.g. chip not connected)
  if ((version > 0) && (version < 0xFF)) {
    return true;
  } else {
    return false;
  }
}

//
// User configuration
//

//****************************************************************
// Funtion Name: setDCFilterOff
// Funtion     : Disable digital DC blocking filter before demodulator
// Input       : BOOLEAN filter true/false
// Output      : none
//****************************************************************
void SmartCC1101::setDCFilterOff(bool dcf) {

  uint8_t state = readRegister(CC1101_MDMCFG2);

  if (dcf == true)
    state |= 0b10000000;
  else
    state &= 0b01111111;

  writeRegister(CC1101_MDMCFG2, state);
}

//****************************************************************
// Funtion Name: setRXBandWitdth
// Funtion     : Set Receive bandwidth in kHz
// Input       : Bandwidth
//	bw_58kHz  = 0b11110000,
//	bw_68kHz  = 0b11100000,
//	bw_81kHz  = 0b11010000,
//	bw_102kHz = 0b11000000,
//	bw_116kHz = 0b10110000,
//	bw_135kHz = 0b10100000,
//	bw_162kHz = 0b10010000,
//	bw_203kHz = 0b10000000,
//	bw_232kHz = 0b01110000,
//	bw_270kHz = 0b01100000,
//	bw_325kHz = 0b01010000,
//	bw_406kHz = 0b01000000,
//	bw_464kHz = 0b00110000,
//	bw_541kHz = 0b00100000,
//	bw_650kHz = 0b00010000,
//	bw_812kHz = 0b00000000
// Output      : none
//****************************************************************
// FIXME: Test!
void SmartCC1101::setRXBandWitdth(rx_BandWidth bw) {

  setIDLEstate();
  uint8_t state = readRegister(CC1101_MDMCFG4);
  state &= 0b00001111;

  writeRegister(CC1101_MDMCFG4, state | bw);
}

//****************************************************************
// Funtion Name: setPA - no power shaping impelemented
// Funtion     : set CC1101 PA Power. Not persisted during sleep, except index 0
//				 For ASK/OOK, PA needs to be set again after sleep
// Input       : PA Power in dB
// Output      : none
//****************************************************************
void SmartCC1101::setPA(int8_t p) {
  uint8_t PA_TABLE[8]{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  int8_t a = 0;
  uint8_t index=0;
  pa = p;

  if (MHz >= 300 && MHz <= 348) {
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
  } else if (MHz >= 378 && MHz <= 464) {
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
  } else if (MHz >= 779 && MHz <= 899.99) {
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
  } else if (MHz >= 900 && MHz <= 928) {
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

//****************************************************************
// Funtion Name: setFrequency
// Funtion     : Set the carrier frequency.
// Input       : Frequency in Hz
// Output      : none
//****************************************************************
void SmartCC1101::setFrequency(const uint32_t freq) {
  // We use uint64_t as the <<16 overflows uint32_t
  // however the division with 26000000 allows the final
  // result to be uint32 again
  uint32_t reg_freq = ((uint64_t)freq << 16) / CC1101_CRYSTAL_FREQUENCY;

  MHz = freq / 1.e6;

  // this is split into 3 bytes that are written to 3 different registers on the CC1101
  uint8_t FREQ2 = (reg_freq >> 16) & 0xFF;  // high byte, bits 7..6 are always 0 for this register
  uint8_t FREQ1 = (reg_freq >> 8) & 0xFF;   // middle byte
  uint8_t FREQ0 = reg_freq & 0xFF;          // low byte
  setIDLEstate();
  writeRegister(CC1101_CHANNR, 0);
  writeRegister(CC1101_FREQ2, FREQ2);
  writeRegister(CC1101_FREQ1, FREQ1);
  writeRegister(CC1101_FREQ0, FREQ0);

}

//****************************************************************
// Funtion Name: setModulation
// Funtion     : set CC1101 Modulation
// Input       : Modulation
//  mod_2FSK
//  mod_GFSK
//  mod_ASKOOK
//  mod_4FSK
//  mod_MSK
// Output      : none
//****************************************************************
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

//
// Transmission configuration
//

//****************************************************************
// Funtion Name: Set PKT_FORMAT
// Funtion     : Format of RX and TX data (this should be "normal" in most cases)
// Input       : Packet format
// pktf_NORMAL    Normal mode, use FIFOs for RX and TX
// pktf_RANDOMTX  Sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
// Output      : none
//****************************************************************
void SmartCC1101::setPktFormat(PacketFormat pktf) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);
  state &= 0b11001111;

  writeRegister(CC1101_PKTCTRL0, state | pktf);
}

//****************************************************************
// Funtion Name: setAdrChk
// Funtion     : Controls address check configuration of received packages
// Input       : Address Check configtuation
//	adc_NO			No address check
//	adc_YESNOBC		Address check, no broadcast
//	adc_YES0BC		Address check and 0 (0x00) broadcast
//	adc_YES0FFBC	Address check and 0 (0x00) and 255 (0xFF) broadcast
// Output      : none
//****************************************************************
void SmartCC1101::setAdrChk(AddressCheck adc) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);
  state &= 0b11111100;

  writeRegister(CC1101_PKTCTRL1, state | adc);
}

//****************************************************************
// Funtion Name: setAddr
// Funtion     : Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
// Input       : Channel number
// Output      : none
//****************************************************************
void SmartCC1101::setAddr(uint8_t addr) {
  writeRegister(CC1101_ADDR, addr);
}

//****************************************************************
// Funtion Name: setCRCCheck
// Funtion     : CRC calculation in TX and CRC check in RX
// Input       : Boolean flag if CRC should be checked
// Output      : none
//****************************************************************
void SmartCC1101::setCRCCheck(bool crc) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);

  if (crc == true)
    state |= 0b00000100;
  else
    state &= 0b11111011;

  writeRegister(CC1101_PKTCTRL0, state);
}

//****************************************************************
// Funtion Name: setCRC_AF
// Funtion     : Enable automatic flush of RX FIFO when CRC is not OK
// Input       : Boolean flag to enabel autoflush
// Output      : none
//****************************************************************
void SmartCC1101::setCRC_AF(bool af) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);

  if (af == true)
    state |= 0b00001000;
  else
    state &= 0b11110111;

  writeRegister(CC1101_PKTCTRL1, state);
}

//****************************************************************
// Funtion Name: Set FEC
// Funtion     : Enable Forward Error Correction (FEC) works only in fxed length mode
// Input       : Boolean flag to enable FEC
// Output      : none
//****************************************************************
void SmartCC1101::setFEC(bool fec) {

  uint8_t state = readRegister(CC1101_MDMCFG1);
  if (fec == true)
    state |= 0b10000000;
  else
    state &= 0b01111111;

  writeRegister(CC1101_MDMCFG1, state);
}


//****************************************************************
// Funtion Name: Set LENGTH_CONFIG
// Funtion     : Configure the packet length
// Input       : Length control mode
// Output      : none
//****************************************************************
void SmartCC1101::setLengthConfig(PacketLengthConfig lenc) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);
  state &= 0b11111100;

  writeRegister(CC1101_PKTCTRL0, state | lenc);
}

//****************************************************************
// Funtion Name: Set PACKET_LENGTH
// Funtion     : Indicates the packet length in fixed length mode
//				 if variable packet length mode is used, this value indicates the maximum
//				 packet length allowed.
// Input       : Packet Length
// Output      : none
//****************************************************************
void SmartCC1101::setPacketLength(uint8_t pktlen) {
  // although 255 bytes are theoretically possible, all code the here is limited
  // to the RXBUFFER/TXBUFFER length of 64 bytes, allowing net 61 bytes of payload
  if (pktlen > 61)
    pktlen = 61;
  writeRegister(CC1101_PKTLEN, pktlen);
}

//****************************************************************
// Funtion Name: setSyncWord
// Funtion     : Sync Word
// Input       : Syncword high, low
// Output      : none
//****************************************************************
void SmartCC1101::setSyncWord(uint8_t sh, uint8_t sl) {
  writeRegister(CC1101_SYNC1, sh);
  writeRegister(CC1101_SYNC0, sl);
}

//****************************************************************
// Funtion Name: Set SYNC_MODE
// Funtion     : Combined sync-word qualifier mode
// Input       : Sync Mode
//	sync_NONE   No preamble/sync
//	sync_1516   15/16 sync word bits detected
//	sync_1616   16/16 sync word bits detected
//	sync_3032   30/32 sync word bits detected
//	sync_NONECS No preamble/sync, carrier-sense above threshold
//	sync_1516CS 15/16 + carrier-sense above threshold
//	sync_1616CS 16/16 + carrier-sense above threshold
//	sync_3032CS 30/32 + carrier-sense above threshold
// Output      : none
//****************************************************************
void SmartCC1101::setSyncMode(sync_Mode syncm) {

  uint8_t state = readRegister(CC1101_MDMCFG2);
  SDEBUGln("CC1101_MDMCFG2 before %0x",state);
  state &= 0b11111000;
  SDEBUGln("CC1101_MDMCFG2 after %0x",state | syncm);

  writeRegister(CC1101_MDMCFG2, state | syncm);
}

//****************************************************************
// Funtion Name: Set PRE
// Funtion     : Sets the minimum number of preamble bytes to be transmitted.
// Input       : Preamble Bytes
//	pre_2
//	pre_3
//	pre_4
//	pre_6
//	pre_8
//	pre_12
//	pre_16
//	pre_24
// Output      : none
//****************************************************************
void SmartCC1101::setPRE(preamble_Bytes pre) {

  uint8_t state = readRegister(CC1101_MDMCFG1);
  state &= 0b10001111;

  writeRegister(CC1101_MDMCFG1, state | pre);
}

//****************************************************************
// Funtion Name: setPQT
// Funtion     : Preamble quality estimator threshold
//               A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted
// Input       : PQT, range from 0-7
// Output      : none
//****************************************************************
void SmartCC1101::setPQT(uint8_t pqt) {

  uint8_t state = readRegister(CC1101_PKTCTRL1);
  state &= 0b00011111;

  pqt <<= 5; // this takes care we do not use alues exceeding range

  writeRegister(CC1101_PKTCTRL1, state | pqt);
}

//****************************************************************
// Funtion Name: Set MANCHESTER
// Funtion     : Enables Manchester encoding/decoding
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::setManchester(bool menc) {

  uint8_t state = readRegister(CC1101_MDMCFG2);
  if (menc == true)
    state |= 0b00001000;
  else
    state &= 0b11110111;

  writeRegister(CC1101_MDMCFG2, state);
}

//****************************************************************
// Funtion Name: setWhiteData
// Funtion     : Turn data whitening on / off.
// Input       : none
// Output      : none
//****************************************************************
void SmartCC1101::setWhiteData(bool white) {

  uint8_t state = readRegister(CC1101_PKTCTRL0);

  if (white == true)
    state |= 0b01000000;
  else
    state &= 0b10111111;

  writeRegister(CC1101_PKTCTRL0, state);
}

//****************************************************************
// Funtion Name: setDRate
// Funtion     : Set RX/TX symbolrate
// Input       : Symbol rate in kbit/s
// Output      : none
//****************************************************************
void SmartCC1101::setDRate(float d) {

  setIDLEstate();
  uint8_t state = readRegister(CC1101_MDMCFG4) & 0b00110000;

  float c = d;

  uint8_t MDMCFG3 = 0;

  if (c > 1621.83) { c = 1621.83; }
  if (c < 0.0247955) { c = 0.0247955; }

  uint8_t m4DaRa = 0;

  for (int i = 0; i < 20; i++) {
    if (c <= 0.0494942) {
      c = c - 0.0247955;
      c = c / 0.00009685;
      MDMCFG3 = c;
      float s1 = (c - MDMCFG3) * 10;
      if (s1 >= 5) { MDMCFG3++; }
      i = 20;
    } else {
      m4DaRa++;
      c = c / 2;
    }
  }
  writeRegister(CC1101_MDMCFG4, state | m4DaRa);
  writeRegister(CC1101_MDMCFG3, MDMCFG3);
}

//****************************************************************
// Funtion Name: setDeviation
// Funtion     : Specifies the nominal frequency deviation from the carrier for a ‘0’ (-DEVIATN) and ‘1’ (+DEVIATN)
//				 for 2-FSK, GFSK, 4-FSK
//				 MSK: Specifies the fraction of symbol period (1/8-8/8) during which a phase change occurs (‘0’: +90deg, ‘1’:-90deg)
// Input       : Deviation in kHz
// Output      : none
//****************************************************************
void SmartCC1101::setDeviation(float d) {
  float f = 1.586914;
  float v = 0.19836425;
  int c = 0;
  if (d > 380.859375) { d = 380.859375; }
  if (d < 1.586914) { d = 1.586914; }
  for (int i = 0; i < 255; i++) {
    f += v;
    if (c == 7) {
      v *= 2;
      c = -1;
      i += 8;
    }
    if (f >= d) {
      c = i;
      i = 255;
    }
    c++;
  }
  writeRegister(CC1101_DEVIATN, c);
}

//****************************************************************
// Funtion Name: sendData 
// Funtion     : use CC1101 send data
// Input       : txBuffer: zero-terminated character array to send, no more than 61
// Output      : none
//****************************************************************
void SmartCC1101::sendData(char *txBuffer) {
  sendData((uint8_t *)txBuffer, (uint8_t)strlen(txBuffer));
}

//****************************************************************
// Funtion Name: SendData
// Funtion     : use CC1101 send data
// Input       : txBuffer: data array to send; size: number of data to send, no more than 61
// Output      : none
//****************************************************************
void SmartCC1101::sendData(uint8_t *txBuffer, uint8_t size) {

  uint8_t state;
 
  if(sleepState)
    onWakeup();
 
  SDEBUGln("Got %d bytes to write: %s", size, txBuffer);

  // limit to 61 characters
  size = (size <= 61) ? size : 61;

  state = getState();
  SDEBUGln("State entering sendData=%d",state);

  setIDLEstate();

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
  uint8_t lock = readRegister(CC1101_FSCAL1);
  if (lock != 0x3f)
    SDEBUGln("Calibration locked FSCAL1 = %0x", readRegister(CC1101_FSCAL1));

  while (1) {
    state = getState();
    SDEBUGln("State = %d", state);
    if (state == state_IDLE) break;  // we wait for IDLE state
    delayMicroseconds(2000);

  }
  
  strobe(CC1101_SFTX);  //flush TXfifo
}

//****************************************************************
// Funtion Name: SetRX
// Funtion     : set CC1101 to receive state
// Input       : none
// Output      : none
//****************************************************************
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
    setIDLEstate();
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

//****************************************************************
// Funtion Name: return RSSI Level
// Funtion     : read current RSSI Level if not IDLE, the one from last RX otherwise
// Input       : none
// Output      : none
//****************************************************************
int8_t SmartCC1101::getRSSI(void) {
  if (getState() != state_IDLE)
    return (getRSSI(readStatusRegister(CC1101_RSSI)));
  else
    return rssi;
}

//****************************************************************
// Funtion Name: return RSSI Level
// Funtion     : Calculating the RSSI Level for a given value
// Input       : raw data readout value from register
// Output      : none
//****************************************************************
int8_t SmartCC1101::getRSSI(uint8_t rawValue) {

  int16_t rssi = rawValue;

  if (rssi >= 128) {
    rssi = (rssi - 256) / 2 - 74;
  } else {
    rssi = (rssi / 2) - 74;
  }
  return (int8_t)rssi;
}

//****************************************************************
// Funtion Name: Check CRC
// Funtion     : read current CRC if not IDLE, the one from last RX otherwise
// Input       : none
// Output      : true if CRC in RX data is ok (or CRC disabled), false otherwise
//****************************************************************
bool SmartCC1101::checkCRC(void) {

  if (getState() != state_IDLE)
    return checkCRC(readStatusRegister(CC1101_LQI));
  else
    return crc;
}

//****************************************************************
// Funtion Name: Check CRC
// Funtion     : calculate CRC for a given raw value
// Input       : raw data readout value from register
// Output      : true if CRC in RX data is ok, false otherwise
//****************************************************************
bool SmartCC1101::checkCRC(uint8_t rawValue) {

  if (rawValue & 0b10000000)
    return true;
  else
    return false;
}

//****************************************************************
// Funtion Name: Link Quality Indication (LQI)
// Funtion     : read current LQI if not IDLE, the one from last RX otherwise
// Input       : none
// Output      : LQI value (lower is better)
//****************************************************************
uint8_t SmartCC1101::getLQI(void) {
  if (getState() != state_IDLE)
    return getLQI(readStatusRegister(CC1101_LQI));
  else
    return lqi;
}

//****************************************************************
// Funtion Name: Link Quality Indication (LQI)
// Funtion     : Calculate LQI for given raw value
// Input       : raw data readout value from register
// Output      : LQI value (lower is better)
//****************************************************************
uint8_t SmartCC1101::getLQI(uint8_t rawValue) {
  return (rawValue & 0b01111111);
}

//****************************************************************
// Funtion Name: receiveData
// Funtion     : read data received from RXfifo
// Input       : rxBuffer: buffer to store data
// Output      : size of data received (0 if still RX'ing, inconsisntent state or CRC error and CRC_AF=true)
//****************************************************************

uint8_t SmartCC1101::receiveData(uint8_t *rxBuffer) {
	
  if(sleepState)
    onWakeup();

  uint8_t size = 0;
  uint8_t state = getState();
  if (state == state_RX) {  // RX not yet complete, will be idle if RX finshes
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
//****************************************************************
// Function Name: dumRegs (debugging helper)
// Funtion      : output register contents to console
// Input        : none
// Output       : none
//****************************************************************

void SmartCC1101::dumpRegs(void) {

  SDEBUGln("CC1101 Register Dump:");
  for (int i = 0; i < 0x40; i++) {
    SDEBUGln("Register %0x: %0x", i, readStatusRegister(i));
  }
}
#endif

SmartCC1101 Smartcc1101;
