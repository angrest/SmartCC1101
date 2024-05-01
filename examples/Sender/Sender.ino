// Memeory Consumption:
// 20240421:
// Der Sketch verwendet 5582 Bytes (18%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
// Globale Variablen verwenden 216 Bytes (10%) des dynamischen Speichers, 1832 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.
// 20240430:
// Der Sketch verwendet 4582 Bytes (14%) des Programmspeicherplatzes. Das Maximum sind 30720 Bytes.
// Globale Variablen verwenden 216 Bytes (10%) des dynamischen Speichers, 1832 Bytes für lokale Variablen verbleiben. Das Maximum sind 2048 Bytes.


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

  Smartcc1101.setDCFilterOff(false);                        // Disable digital DC blocking filter before demodulator. Only for data rates ≤ 250 kBaud. The recommended IF frequency changes when the DC blocking is disabled. true = Disable (current optimized), false = Enable (better sensitivity).
  Smartcc1101.setModulation(SmartCC1101::mod_2FSK);         // set modulation mode. Possible values: mod_2FSK, mod_GFSK, mod_ASKOOK, mod_4FSK, mod_MSK
  Smartcc1101.setCarrierFrequency(868350000);               // Set tranmsision frequency in Hz. Default = 868.35 MHz). The cc1101 can use 300-348 MHZ, 387-464MHZ and 779-928MHZ. More info in the datasheet.
  Smartcc1101.setPA(12);                                    // Set TXPower. The following settings are possible depending on the frequency band.  (-30  -20  -15  -10  -6    0    5    7    10   11   12) Default is max!
  Smartcc1101.setDeviation(47608);                          // Set the Frequency deviation in Hz. Value from 1586 to 380850. Default is 47607 kHz. Note: the algorithm rounds down. On exact interval limits, the next lower vlue might be taken.
  Smartcc1101.setRXBandWitdth(SmartCC1101::bw_812kHz);      // Set the Receive Bandwidth in kHz. Possible values: bw_58kHz  = 58kHz, bw_68kHz, bw_81kHz, bw_102kHz,bw_116kHz, bw_135kHz, bw_162kHz, bw_203kHz,	bw_232kHz, bw_270kHz,	bw_325kHz, bw_406kHz,	bw_464kHz, bw_541kHz,	bw_650kHz, bw_812kHz = 812kHz
  Smartcc1101.setSymbolRate(100000);                        // Set the Data Rate in Baud. Value from 20 to 1621830 Default is 115051 Baud
                                                            // Smartcc1101.setSyncWord(W_CCSYNC_LOW, W_CCSYNC_HIGH);    // Set sync word. Must be the same for the transmitter and receiver. (Syncword high, Syncword low)
  Smartcc1101.setSyncMode(SmartCC1101::sync_1616);          // Combined sync-word qualifier mode. 0 = No preamble/sync. 1 = 15/16 sync word bits detected. 2 = 16/16 sync word bits detected. 3 = 30/32 sync word bits detected. 4 = No preamble/sync, carrier-sense above threshold. 5 = 15/16 + carrier-sense above threshold. 6 = 16/16 + carrier-sense above threshold. 7 = 30/32 + carrier-sense above threshold.
  Smartcc1101.setPRE(SmartCC1101::pre_4);                   // Sets the minimum number of preamble bytes to be transmitted. Possible values: pre_2 : 2, pre_3 : 3, pre_4 : 4, pre_6 : 6, pre_8 : 8, pre_12 : 12, pre_16 : 16, pre_24 : 24
  Smartcc1101.setPQT(0);                                    // Preamble quality estimator threshold. The preamble quality estimator increases an internal counter by one each time a bit is received that is different from the previous bit, and decreases the counter by 8 each time a bit is received that is the same as the last bit. A threshold of 4∙PQT for this counter is used to gate sync word detection. When PQT=0 a sync word is always accepted.
  Smartcc1101.setAdrChk(SmartCC1101::adc_NO);               // Controls address check configuration of received packages. adc_NO = No address check. adc_YESNOBC = Address check, no broadcast. adc_YES0BC = Address check and 0 (0x00) broadcast. adc_YES0FFBC = Address check and 0 (0x00) and 255 (0xFF) broadcast.
  Smartcc1101.setAddr(0);                                   // Address used for packet filtration. Optional broadcast addresses are 0 (0x00) and 255 (0xFF).
  Smartcc1101.setManchester(true);                          // Enables Manchester encoding/decoding. false = Disable, true = Enable.
  Smartcc1101.setWhiteData(true);                           // Turn data whitening on / off. 0 = Whitening off. 1 = Whitening on.
  Smartcc1101.setPktFormat(SmartCC1101::pktf_NORMAL);       // Format of RX and TX data. pktf_NORMAL = Normal mode, use FIFOs for RX and TX. pktf_RANDOMTX = Random TX mode; sends random data using PN9 generator. Used for test. Works as normal mode, setting 0 (00), in RX.
  Smartcc1101.setLengthConfig(SmartCC1101::pktl_VARIABLE);  // pktl_FIXED = Fixed packet length mode. pktl_VARIABLE = Variable packet length mode.
  Smartcc1101.setPacketLength(61);                          // Indicates the packet length when fixed packet length mode is enabled. If variable packet length mode is used, this value indicates the maximum packet length allowed. 61 is maximum allowed value in this implementation
  Smartcc1101.setFEC(false);                                // Enable Forward Error Correction (FEC) with interleaving for packet payload (Only supported for fixed packet length mode. false = Disable, true = Enable.
  Smartcc1101.setCRCCheck(true);                            // true = CRC calculation in TX and CRC check in RX enabled. false = CRC disabled for TX and RX.
  Smartcc1101.setCRC_AF(false);                             // Enable automatic flush of RX FIFO when CRC is not OK.
}

void loop() {

  const char messageText[] = "Hello world!";
  // Send data
  Smartcc1101.sendData(messageText);

  delay(10000);

}
