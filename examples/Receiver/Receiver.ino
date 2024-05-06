/**
* @brief Example for CC1101 module library
* @file Receiver.ino
* @author Axel Grewe
* 
* This is the implementation of a basic receiver
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