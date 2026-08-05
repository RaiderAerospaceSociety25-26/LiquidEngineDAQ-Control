#include <SPI.h>
#include "Wire.h"


// THERMISTOR ADC (ADS1115)
#include "Adafruit_ADS1X15.h"
#define TM_SDA 18 // need pin number
#define TM_SCL 19 // need pin number
Adafruit_ADS1115 TM; //define thermistor

// PGA settings for thermistors
#define ADS1X15_REG_CONFIG_PGA_MASK (0x0E00)   ///< PGA Mask
#define ADS1X15_REG_CONFIG_PGA_6_144V (0x0000) ///< +/-6.144V range = Gain 2/3
#define ADS1X15_REG_CONFIG_PGA_4_096V (0x0200) ///< +/-4.096V range = Gain 1
#define ADS1X15_REG_CONFIG_PGA_2_048V (0x0400) ///< +/-2.048V range = Gain 2 (default)
#define ADS1X15_REG_CONFIG_PGA_1_024V (0x0600) ///< +/-1.024V range = Gain 4
#define ADS1X15_REG_CONFIG_PGA_0_512V (0x0800) ///< +/-0.512V range = Gain 8
#define ADS1X15_REG_CONFIG_PGA_0_256V (0x0A00) ///< +/-0.256V range = Gain 16

// data rate settings for thermistors
#define RATE_ADS1115_8SPS (0x0000)   ///< 8 samples per second
#define RATE_ADS1115_16SPS (0x0020)  ///< 16 samples per second
#define RATE_ADS1115_32SPS (0x0040)  ///< 32 samples per second
#define RATE_ADS1115_64SPS (0x0060)  ///< 64 samples per second
#define RATE_ADS1115_128SPS (0x0080) ///< 128 samples per second (default)
#define RATE_ADS1115_250SPS (0x00A0) ///< 250 samples per second
#define RATE_ADS1115_475SPS (0x00C0) ///< 475 samples per second
#define RATE_ADS1115_860SPS (0x00E0) ///< 860 samples per second


// LOAD CELL ADC (NAU7802)
#include "Adafruit_NAU7802.h"
#define LC_SDA 0; //need pin number
#define LC_SCL 0; //need pin number
Adafruit_NAU7802 LC;


// setup function
void setup() {
  // THERMISTOR SETUP
  TM.begin(0x48);
  TM.setGain(ADS1X15_REG_CONFIG_PGA_4_096V);
  TM.setDataRate(RATE_ADS1115_250SPS); //0x0080 is 128 Hz; 0x00A0 is 250 Hz; 0x00C0 is 475 Hz; 0x00E0 is 860 Hz
  int16_t TM1_reading = TM.readADC_Differential_0_1();
  int16_t TM2_reading = TM.readADC_Differential_2_3(); //might want to use single-ended to read all four separately, but not sure yet
  //int16_t TM_reading = PT.readADC_SingleEnded(0); //single-ended reading; index goes from 0 to 3

  // LC SETUP
  // default I2C address is 0x2A
  LC.begin();
  LC.enable(True); // turns sensor enabled and working
  LC.setLDO(NAU7802_4V5); //sets excitation voltage (set it as high as it can go: 4.5 V)
  LC.setGain(NAU7802_GAIN_4); //sets PGA level
  LC.setRate(NAU7802_RATE_80SPS); //sets sample rate (highest is 320?)
  //LC.calibrate(); //performs internal calibration; don't know if this is necessary; also need a calibration mode
  int32_t LC_reading = LC.read(); //read value

}


void loop() {

  
}