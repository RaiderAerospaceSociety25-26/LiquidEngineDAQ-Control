#include <SPI.h>

// ----------------------------- ADS1256 (8-CHANNEL ADC FOR PRESSURE TRANSDUCERS) -----------------------------------
#include <ADS1256.h> // library documentation: https://github.com/CuriousScientist0/ADS1256/blob/main/extras/ADS1256_ArduinoLibrary_Documentation_20251023.pdf

// ---------------- HELPER DEFINITIONS ----------------
//Differential inputs
// #define DIFF_0_1 0b00000001 //A0 + A1 as differential input
// #define DIFF_2_3 0b00100011 //A2 + A3 as differential input
// #define DIFF_4_5 0b01000101 //A4 + A5 as differential input
// #define DIFF_6_7 0b01100111 //A6 + A7 as differential input

//Single-ended inputs
// #define SING_0 0b00001111 //A0 + GND (common) as single-ended input
// #define SING_1 0b00011111 //A1 + GND (common) as single-ended input
// #define SING_2 0b00101111 //A2 + GND (common) as single-ended input
// #define SING_3 0b00111111 //A3 + GND (common) as single-ended input
// #define SING_4 0b01001111 //A4 + GND (common) as single-ended input
// #define SING_5 0b01011111 //A5 + GND (common) as single-ended input
// #define SING_6 0b01101111 //A6 + GND (common) as single-ended input
// #define SING_7 0b01111111 //A7 + GND (common) as single-ended input

//PGA settings			  //Input voltage range
#define PGA_1 0b00000000  //± 5 V
#define PGA_2 0b00000001  //± 2.5 V
#define PGA_4 0b00000010  //± 1.25 V
#define PGA_8 0b00000011  //± 625 mV
#define PGA_16 0b00000100 //± 312.5 mV
#define PGA_32 0b00000101 //+ 156.25 mV
#define PGA_64 0b00000110 //± 78.125 mV

//Datarate						  //DEC
#define DRATE_30000SPS 0b11110000 //240
#define DRATE_15000SPS 0b11100000 //224
#define DRATE_7500SPS 0b11010000  //208
#define DRATE_3750SPS 0b11000000  //192
#define DRATE_2000SPS 0b10110000  //176
#define DRATE_1000SPS 0b10100001  //161
#define DRATE_500SPS 0b10010010   //146
#define DRATE_100SPS 0b10000010   //130
#define DRATE_60SPS 0b01110010    //114
#define DRATE_50SPS 0b01100011    //99
#define DRATE_30SPS 0b01010011    //83
#define DRATE_25SPS 0b01000011    //67
#define DRATE_15SPS 0b00110011    //51
#define DRATE_10SPS 0b00100011    //35
#define DRATE_5SPS 0b00010011     //19
#define DRATE_2SPS 0b00000011     //3

//Status register
#define BITORDER_MSB 0
#define BITORDER_LSB 1
#define ACAL_DISABLED 0
#define ACAL_ENABLED 1
#define BUFFER_DISABLED 0
#define BUFFER_ENABLED 1

//Register addresses
#define STATUS_REG 0x00
#define MUX_REG 0x01
#define ADCON_REG 0x02
#define DRATE_REG 0x03
#define IO_REG 0x04
#define OFC0_REG 0x05
#define OFC1_REG 0x06
#define OFC2_REG 0x07
#define FSC0_REG 0x08
#define FSC1_REG 0x09
#define FSC2_REG 0x0A

//Command definitions
#define WAKEUP 0b00000000
#define RDATA 0b00000001
#define RDATAC 0b00000011
#define SDATAC 0b00001111
#define RREG 0b00010000
#define WREG 0b01010000
#define SELFCAL 0b11110000
#define SELFOCAL 0b11110001
#define SELFGCAL 0b11110010
#define SYSOCAL 0b11110011
#define SYSGCAL 0b11110100
#define SYNC 0b11111100
#define STANDBY 0b11111101
#define RESET 0b11111110
//--------------------------------

//pin definitions
#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13
#define USE_SPI SPI
#define DRDY 7
#define SYNC 8
#define CS 10
#define VREF_M = 2.500 //measured VREF value

#define BUF_SET = BUFFER_DISABLED //Set buffer off (0) or on (1). BUFFER MUST BE TURNED OFF FOR READING +4.5V FROM RATIOMETRIC PT!
#define PGA_SET PGA_1 //Set PGA (programmable gain amplifier) to 1 to measure ± 5 V
//Note: datarate SPS numbers are only for single-input measurement. If more inputs are used, above 100 SPS, the true SPS is much lower due to the multiplexer having to cycle through inputs: 30000 SPS = 4374 Hz, 2000 SPS = 1438 Hz, 500 SPS = 456 Hz, and so on.
#define DRATE_SET DRATE_2000SPS //set to 2000 sps for now
//int singleEndedChannels[8] = {SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7}; //Array to store the single-ended channels
//------------------------------------------------------------------------------------------------------------------


void setup() {

  // ----------------------------- INITIALIZE ADS1256 -----------------------------------
  //Create ADS1256 object. (DRDY, RESET, SYNC(PDWN), CS, VREF(float)) [for Teensy 4.0]. Measure VREF; if it is not 2.500 exactly, measure it with a multimeter and input it correctly here; this is essential for accurachy.
  ADS1256 PT_ADC(DRDY, ADS1256::PIN_UNUSED, SYNC, CS, VREF_M, &USE_SPI);

  //settings
  PT_ADC.setBuffer(BUFFER_DISABLED) //set buffer; ensure buffer is turned off for reading +4.5V from ratiometric PT
  uint8_t buffer_val = PT_ADC.getBuffer() //get buffer
  PT_ADC.setPGA(PGA_SET) //set PGA
  PT_ADC.setDRATE(DRATE_SET) //set data rate

  //read
  PT_ADC.cycleSingle() //cycleSingle() returns a 24 bit signed value converted to a long, and cycles through channels 0 -> 7 (multiplexing); as far as I can tell, each call of this function returns the long value of a single channel, and it just loops through the channels with each call of this function (not entirely sure though)
  PT_ADC.stopConversion() //conversion must be stopped after reading is no longer necessary
  // ------------------------------------------------------------------------------------

}

void loop() {
  

}
