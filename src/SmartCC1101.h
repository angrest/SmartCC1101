/**
* @brief CC1101 module library
* @file SmartCC1101.h
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

#ifndef SmartCC1101_SRC_DRV_h
#define SmartCC1101_SRC_DRV_h

#include <Arduino.h>
#include <SPI.h>


//#define DEBUG_CC1101

/**
* Some debugging helpers
*/
#ifdef DEBUG_CC1101
#ifndef __FILENAME__
#ifdef ESP32
#define __FILENAME__ strrchr("/" __FILE__, '/') + 1
#else
#define __FILENAME__ __FILE__
#endif
#endif

#if defined (ESP32) || defined (ESP8266)
#define SDEBUGln(format, ...) {Serial.printf( "[%6lu][%s:%u] %s(): " format "\n", (unsigned long) (millis() / 1000ULL), __FILENAME__, __LINE__, __FUNCTION__, ##__VA_ARGS__); Serial.flush();}
#else
#define SDEBUGln(format, ...) {char _Dbuf[80]; snprintf_P(_Dbuf,sizeof(_Dbuf), PSTR(format), ##__VA_ARGS__); Serial.println(_Dbuf);Serial.flush();}
#endif
#define SDEBUG(text, ...) Serial.print(text ##__VA_ARGS__)
#else
#define SDEBUGln(format, ...) {}
#define SDEBUG(text, ...) {}
#endif


#include <Arduino.h>

/**
* define SPI pins - CS pin should be user configurable
*/
#if defined __AVR_ATmega168__ || defined __AVR_ATmega328P__
#define    SCK_PIN 13
#define    CIPO_PIN 12
#define    COPI_PIN 11
#define    CS_PIN 10
#elif defined __AVR_ATmega1280__ || defined __AVR_ATmega2560__
#define    SCK_PIN 52
#define    CIPO_PIN 50
#define    COPI_PIN 51
#define    CS_PIN 53
#elif ESP8266
#define    SCK_PIN 14
#define    CIPO_PIN 12
#define    COPI_PIN 13
#define    CS_PIN 15
#elif ESP32
#define    SCK_PIN 18
#define    CIPO_PIN 19
#define    COPI_PIN 23
#define    CS_PIN 5
#else
#define    SCK_PIN 13
#define    CIPO_PIN 12
#define    COPI_PIN 11
#define    CS_PIN 10
#endif


/**
* CC1101 register shortcuts
*/
#define CC1101_IOCFG2 0x00    // GDO2 output pin configuration
#define CC1101_IOCFG1 0x01    // GDO1 output pin configuration
#define CC1101_IOCFG0 0x02    // GDO0 output pin configuration
#define CC1101_FIFOTHR 0x03   // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1 0x04     // Sync word, high INT8U
#define CC1101_SYNC0 0x05     // Sync word, low INT8U
#define CC1101_PKTLEN 0x06    // Packet length
#define CC1101_PKTCTRL1 0x07  // Packet automation control
#define CC1101_PKTCTRL0 0x08  // Packet automation control
#define CC1101_ADDR 0x09      // Device address
#define CC1101_CHANNR 0x0A    // Channel number
#define CC1101_FSCTRL1 0x0B   // Frequency synthesizer control
#define CC1101_FSCTRL0 0x0C   // Frequency synthesizer control
#define CC1101_FREQ2 0x0D     // Frequency control word, high INT8U
#define CC1101_FREQ1 0x0E     // Frequency control word, middle INT8U
#define CC1101_FREQ0 0x0F     // Frequency control word, low INT8U
#define CC1101_MDMCFG4 0x10   // Modem configuration
#define CC1101_MDMCFG3 0x11   // Modem configuration
#define CC1101_MDMCFG2 0x12   // Modem configuration
#define CC1101_MDMCFG1 0x13   // Modem configuration
#define CC1101_MDMCFG0 0x14   // Modem configuration
#define CC1101_DEVIATN 0x15   // Modem deviation setting
#define CC1101_MCSM2 0x16     // Main Radio Control State Machine configuration
#define CC1101_MCSM1 0x17     // Main Radio Control State Machine configuration
#define CC1101_MCSM0 0x18     // Main Radio Control State Machine configuration
#define CC1101_FOCCFG 0x19    // Frequency Offset Compensation configuration
#define CC1101_BSCFG 0x1A     // Bit Synchronization configuration
#define CC1101_AGCCTRL2 0x1B  // AGC control
#define CC1101_AGCCTRL1 0x1C  // AGC control
#define CC1101_AGCCTRL0 0x1D  // AGC control
#define CC1101_WOREVT1 0x1E   // High INT8U Event 0 timeout
#define CC1101_WOREVT0 0x1F   // Low INT8U Event 0 timeout
#define CC1101_WORCTRL 0x20   // Wake On Radio control
#define CC1101_FREND1 0x21    // Front end RX configuration
#define CC1101_FREND0 0x22    // Front end TX configuration
#define CC1101_FSCAL3 0x23    // Frequency synthesizer calibration
#define CC1101_FSCAL2 0x24    // Frequency synthesizer calibration
#define CC1101_FSCAL1 0x25    // Frequency synthesizer calibration
#define CC1101_FSCAL0 0x26    // Frequency synthesizer calibration
#define CC1101_RCCTRL1 0x27   // RC oscillator configuration
#define CC1101_RCCTRL0 0x28   // RC oscillator configuration
#define CC1101_FSTEST 0x29    // Frequency synthesizer calibration control
#define CC1101_PTEST 0x2A     // Production test
#define CC1101_AGCTEST 0x2B   // AGC test
#define CC1101_TEST2 0x2C     // Various test settings
#define CC1101_TEST1 0x2D     // Various test settings
#define CC1101_TEST0 0x2E     // Various test settings

/**
* Strobe commands
*/
#define CC1101_SRES 0x30     // Reset chip.
#define CC1101_SFSTXON 0x31  // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                             // If in RX/TX: Go to a wait state where only the synthesizer is 
                             // running (for quick RX / TX turnaround).
#define CC1101_SXOFF 0x32    // Turn off crystal oscillator.
#define CC1101_SCAL 0x33     // Calibrate frequency synthesizer and turn it off 
                             // (enables quick start).
#define CC1101_SRX 0x34      // Enable RX. Perform calibration first if coming from IDLE and 
                             // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX 0x35      // In IDLE state: Enable TX. Perform calibration first if 
                             // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled 
                             // Only go to TX if channel is clear.
#define CC1101_SIDLE 0x36    // Exit RX / TX, turn off frequency synthesizer and exit 
                             // Wake-On-Radio mode if applicable.
#define CC1101_SAFC 0x37     // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR 0x38     // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD 0x39     // Enter power down mode when CSn goes high.
#define CC1101_SFRX 0x3A     // Flush the RX FIFO buffer.
#define CC1101_SFTX 0x3B     // Flush the TX FIFO buffer.
#define CC1101_SWORRST 0x3C  // Reset real time clock.
#define CC1101_SNOP 0x3D     // No operation. May be used to pad strobe commands to two 
                             // INT8Us for simpler software.
/**
* status register values
*/
#define CC1101_PARTNUM 0x30
#define CC1101_VERSION 0x31
#define CC1101_FREQEST 0x32
#define CC1101_LQI 0x33
#define CC1101_RSSI 0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_WORTIME1 0x36
#define CC1101_WORTIME0 0x37
#define CC1101_PKTSTATUS 0x38
#define CC1101_VCO_VC_DAC 0x39
#define CC1101_TXBYTES 0x3A
#define CC1101_RXBYTES 0x3B

/**
* PATABLE,TXFIFO,RXFIFO
*/
#define CC1101_PATABLE 0x3E
#define CC1101_TXFIFO 0x3F
#define CC1101_RXFIFO 0x3F

/**
* read/write opions
*/
#define WRITE_BURST 0x40      //write burst
#define READ_SINGLE 0x80      //read single
#define READ_BURST 0xC0       //read burst
#define BYTES_IN_RXFIFO 0x3F  //max bytes in RXfifo

/**
* Most modules come with 26Mhz crystal
*/
#ifndef CC1101_CRYSTAL_FREQUENCY
#define  CC1101_CRYSTAL_FREQUENCY 26000000ul
#endif

/**
* Class definition
*/
class SmartCC1101 {

public:
  enum Modulation: uint8_t {
	mod_2FSK   = 0b00000000,
	mod_GFSK   = 0b00010000,
	mod_ASKOOK = 0b00110000,
	mod_4FSK   = 0b01000000,
	mod_MSK    = 0b01110000
  };
  enum AddressCheck: uint8_t {
	adc_NO,					// No address check
	adc_YESNOBC,			// Address check, no broadcast
	adc_YES0BC,				// Address check and 0 (0x00) broadcast
	adc_YES0FFBC			// Address check and 0 (0x00) and 255 (0xFF) broadcast
  };
  enum PacketFormat: uint8_t {
/*
* @note Synchronous serial mode and Asynchronous serial mode are unsupported
*/
	pktf_NORMAL   = 0b00000000,		// Normal mode, use FIFOs for RX and TX
	pktf_RANDOMTX = 0b00100000		// sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX
  };
  enum PacketLengthConfig: uint8_t {
/**
* @note CC1101 also supports infinite packet lenth mode, but this is not implemented.
*/
	pktl_FIXED,
	pktl_VARIABLE

  };
  enum sync_Mode: uint8_t {
	sync_NONE =   0b000, // No preamble/sync
	sync_1516 =   0b001, // 15/16 sync word bits detected
	sync_1616 =   0b010, // 16/16 sync word bits detected
	sync_3032 =   0b011, // 30/32 sync word bits detected
	sync_NONECS = 0b100, // No preamble/sync, carrier-sense above threshold
	sync_1516CS = 0b101, // 15/16 + carrier-sense above threshold
	sync_1616CS = 0b110, // 16/16 + carrier-sense above threshold 
	sync_3032CS = 0b111  // 30/32 + carrier-sense above threshold
  };
  enum preamble_Bytes: uint8_t {
	pre_2 =  0b00000000, // 2 preamble bytes
	pre_3 =  0b00010000, // 3 preamble bytes
	pre_4 =  0b00100000, // 4 preamble bytes
	pre_6 =  0b00110000, // 6 preamble bytes
	pre_8 =  0b01000000, // 8 preamble bytes
	pre_12 = 0b01010000, // 12 preamble bytes
	pre_16 = 0b01100000, // 16 preamble bytes
	pre_24 = 0b01110000  // 24 preamble bytes
  };
  enum rx_BandWidth: uint8_t {
	bw_58kHz  = 0b11110000,
	bw_68kHz  = 0b11100000,
	bw_81kHz  = 0b11010000,
	bw_102kHz = 0b11000000,
	bw_116kHz = 0b10110000,
	bw_135kHz = 0b10100000,
	bw_162kHz = 0b10010000,
	bw_203kHz = 0b10000000,
	bw_232kHz = 0b01110000,
	bw_270kHz = 0b01100000,
	bw_325kHz = 0b01010000,
	bw_406kHz = 0b01000000,
	bw_464kHz = 0b00110000,
	bw_541kHz = 0b00100000,
	bw_650kHz = 0b00010000,
	bw_812kHz = 0b00000000
  };
  void setDelayFunction(void delayFunc(uint8_t));
  void smartDelay(uint8_t ms);
  void init(void);
  bool getCC1101(void);
  void sleep(void);
  void onWakeup(void);
  void setDCFilterOff(bool dcf);
  void setCarrierFrequency(uint32_t freq);
  void setRXBandWitdth(rx_BandWidth bw);
  void setModulation(Modulation m);
  void setPA(int8_t pa);
  void setDeviation(uint32_t deviation);
  void setSyncMode(sync_Mode syncm);
  void setSyncWord(uint8_t sh, uint8_t sl);
  void setPRE(preamble_Bytes pre);
  void setPQT(uint8_t pqt);
  void setAdrChk(AddressCheck adc);
  void setAddr(uint8_t addr);
  void setPktFormat(PacketFormat pktf);
  void setCRCCheck(bool crcc);
  void setCRC_AF(bool af);
  void setFEC(bool fec);
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

#ifdef DEBUG_CC1101
  void dumpRegs(void);
#endif

private:
  void waitCIPO(void);
  void chipSelect(void);
  void chipDeselect(void);
  uint8_t readRegister(uint8_t addr);
  void readBurstRegister(uint8_t addr, uint8_t *buffer, uint8_t num);
  uint8_t readStatusRegister(uint8_t addr);
  enum chipState: uint8_t {
   state_IDLE			= 0b000,
   state_RX				= 0b001,
   state_TX				= 0b010,
   state_FSTXON			= 0b011,
   state_CALIBRATE		= 0b100,
   state_SETTLING		= 0b101,
   state_RXFIFO_OVERFLOW = 0b110,
   state_TXFIFO_UNDERFLOW = 0b111
  };
  chipState getState(void);
  void writeRegister(uint8_t addr, uint8_t value, const char* str = __builtin_FUNCTION());
  void writeBurstRegister(uint8_t addr, const uint8_t *buffer, uint8_t num, const char* str = __builtin_FUNCTION());
  uint8_t strobe(uint8_t strobe);
  void setIDLEState(void);
  void reset(void);
  void configCC1101(void);
  void calibrate(void);
  int8_t getRSSI(uint8_t rawValue);
  bool checkCRC(uint8_t rawValue);
  uint8_t getLQI(uint8_t rawValue);

  enum Modulation modulation = mod_2FSK;
  uint8_t frend0;
  int8_t pa = 12;
  bool sleepState = false;
  uint32_t cfreq = 886350000;
  bool crc;
  uint8_t lqi;
  int8_t rssi;
  
  typedef void delayfunction(uint8_t); 
  delayfunction *delayFunc = NULL;

#ifdef SPI_HAS_TRANSACTION
  SPISettings mySPISettings;
#endif

};

/**
* Create instance for convenience
*/

extern SmartCC1101 Smartcc1101;

#endif
