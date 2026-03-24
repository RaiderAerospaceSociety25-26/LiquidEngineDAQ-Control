# LiquidEngineDAQ-Control
Repository for DAQ and control system for the liquid engine and test stand being developed by the Raider Aerospace Society (RAS) PigeonWorks Liquid Engine Team during the 2025-2026 academic year.

# Firmware
Basics of PlatformIO for Teensy [here](https://forum.pjrc.com/index.php?threads/tutorial-how-to-use-platformio-visual-code-studio-for-teensy.66674/)

Planning to use PlatformIO for both Teensy and Nucleo144 board.

There is an STM32 plugin for VSCode with info [here](https://www.st.com/content/st_com/en/campaigns/stm32-vs-code-extension-z11.html)

The radios work via Meshtastic (only using TX/RX pins on the boards as far as I know)

## Firmware Architecture
Plan is to just use the common setup and loop structure for the Nucleo to start and the Teensys as well. Ideally move to something like FreeRTOS for the Nucleo board eventually to potentially speed things up.

To install a library in VSCode PlatformIO: Go to PlatformIO Home -> Libraries. Then search for the library and add it. This will add the library as a reference in the `lib_deps` section of the `platformio.ini` file and will add the library to the `<Project>/.pio/libdeps/<chipname>/` folder. Libraries are generally installed per-project like this unless configured otherwise. For already-included libraries (such as `Wire` or `SPI` for the Teensy), you can simply add the library definition directly to the `lib_deps` section (I think?), or maybe this is not needed and you can just do the usual `#include` entry at the top of your code. (If needed, you can directly reference the library GitHub in the `lib_deps` section instead, and this will install the library in the `.pio` folder.

### MCU Function Divisions and Required Libraries
#### Teensy LC/TC
* Libraries:
  * SPI
  * ADS1115
  * NAU7802
* Functions:
  * Obtain thrust and TC data and send it to Nucleo via SPI.
#### Teensy PT
* Libraries:
  * SPI
  * ADS1256
* Functions:
  * Obtain PT data and send it to Nucleo via SPI.
#### Nucleo
* Libraries:
  * SPI.h
    * For SPI communication
  * Wire.h
    * For I2C communication
  * SD.h
    * For writing to SD card
  * Adafruit_PWMServoDriver.h
    * For servo control
* Functions:
  * Receive LC/TC/PT data via two SPI connections
  * Write data to SD card
  * Actuate valves based on predetermined sequences triggered by T-Beam radio over TX/RX pins
  * Sense overpressures or other issues and send alarms and/or autonomously actuate valves

# Hardware
### ADS1256 Notes
ADS1256 analog signal input maxes out at AVDD minus 2V (so 3V maximum analog signal) if using self-calibration routines with the buffer. If we have the capability to perform calibration and write calibration values directly to the register, we could bypass this and use the full range (analog signals accepted up to AVDD), but this would likely be fairly difficult. HOWEVER, it looks like the buffer is the issue. The buffer increases input impedance to lower noise/settling time/do other stuff I don't fully understand. Overall, the buffer is needed for high-output impedance sensors. With our pressure transducers, we have a 0.5V-4.5V signal. Generally, pressure transducers are relatively low impedance (mA PTs are very low impedance, mV PTs are higher impedance I think). Briefly looking up PTs, it appears that their impedances are on the order of 100 Ω, which should have no issue as the input impedance on the ADS1256, even with the buffer off, is (150 kΩ / PGA) for PGA values of 16 or lower (and we won't need a high PGA value).  Therefore, I don't think the buffer is essential. So we're good, we just have to make sure that the buffer stays off if we're measuring pressures higher than ~60%-70% of the range of the pressure transducer. Note that the ADC will require a lot more current with the buffer off. Note that the buffer is actually off by default too. Also make sure to keep the PGA low (which won't be an issue because the signal we're reading is already 0.5V-4.5V).

### T-Beam Notes
RX of T-Beam is connected to TX pin on Arduino/Nucleo, and TX of T-Beam is connected to RX pin on Arduino/Nucleo.

# Docs
Pin assignments for Nucleo board are listed in the manual pg. ~38

# Operation Notes
### Data/Debugging/Flashing MCUs While Under External Power
The 5V/USB power connector trace has been cut for both Teensys (this separates USB and external power). This allows USB to be connected for data only, while the board is powered with external 5V at the same time.

For the Nucleo-144 board, follow instructions on pg. 22 (section 7.4.6) of the manual: connect the USB cable only after the board has been powered on via external power.
