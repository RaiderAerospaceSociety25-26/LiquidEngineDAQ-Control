# LiquidEngineDAQ-Control
Repository for DAQ and control system for the liquid engine and test stand being developed by Kyle Markel for the Raider Aerospace Society (RAS) PigeonWorks Liquid Engine Team during the 2025-2026 and 2026-2027 academic years.

# Firmware
## Firmware Architecture
Plan is to just use Arduino and the common setup and loop structure for the Teensys. The Nucleo will need STM32Cube IDE to be able to flash to both the M7 and M4 chip (there are other ways, but this is the easiest I think). Ideally move to something like FreeRTOS (i.e. X-CUBE-FREERTOS) for the Nucleo board eventually to potentially speed things up.

SPI communication code examples [link](https://www.makerguides.com/master-slave-spi-communication-arduino/#SPI_Master-Slave_Interfacing)

### MCU Function Divisions and Required Libraries
#### Teensy LC/TM
* Functions:
  * Obtain thrust (i.e. load cell) and thermistor data and send it to Nucleo via SPI.
#### Teensy PT
* Functions:
  * Obtain PT data and send it to Nucleo via SPI.
#### Nucleo
* Functions:
  * Receive LC/TM/PT data via two SPI connections
  * Write data to SD card
  * Actuate valves (servo-actuated ball valves) and send ignition signal based on predetermined sequences triggered by T-Beam radio over TX/RX pins
  * Sense overpressures or other issues and send alarms and/or autonomously actuate valves
  * (Maybe: execute automatic safing/depressurization procedures)

# Hardware and Firmware

### MCUs

#### Nucleo-H755ZI-Q
[STMicroelectronics Nucleo-144 H755ZI-Q](https://www.st.com/en/evaluation-tools/nucleo-h755zi-q.html)
Main MCU: STM32H755ZIT6
Secondary MCU: STM32F723IEK6 (I think)
Roughly following this tutorial for coding the STM32: STM32Cube IDE tutorial for multiple cores [link](https://blog.embeddedexpert.io/?p=4075) 

### ADS1256 Notes
ADS1256 analog signal input maxes out at AVDD minus 2V (so 3V maximum analog signal) if using self-calibration routines with the buffer. If we have the capability to perform calibration and write calibration values directly to the register, we could bypass this and use the full range (analog signals accepted up to AVDD), but this would likely be fairly difficult. HOWEVER, it looks like the buffer is the issue. The buffer increases input impedance to lower noise/settling time/do other stuff I don't fully understand. Overall, the buffer is needed for high-output impedance sensors. With our pressure transducers, we have a 0.5V-4.5V signal. Generally, pressure transducers are relatively low impedance (mA PTs are very low impedance, mV PTs are higher impedance I think). Briefly looking up PTs, it appears that their impedances are on the order of 100 Ω, which should have no issue as the input impedance on the ADS1256, even with the buffer off, is (150 kΩ / PGA) for PGA values of 16 or lower (and we won't need a high PGA value).  Therefore, I don't think the buffer is essential. So we're good, we just have to make sure that the buffer stays off if we're measuring pressures higher than ~60%-70% of the range of the pressure transducer. Note that the ADC will require a lot more current with the buffer off. Note that the buffer is actually off by default too. Also make sure to keep the PGA low (which won't be an issue because the signal we're reading is already 0.5V-4.5V).

### T-Beam Notes
RX of T-Beam is connected to TX pin on Arduino/Nucleo, and TX of T-Beam is connected to RX pin on Arduino/Nucleo.
Works via Meshtastic.

### ADS1115 Notes
ADS1115 will be for the thermistors. Just need two resistors of known/measured resistance to create a voltage divider circuit on the breadboard to get the resistance value of the thermistor, and this value is known at a bunch of different temperatures, so the temperature can be found easily in this manner.
Thermistor link [here](https://www.automationdirect.com/adc/shopping/catalog/process_control_-a-_measurement/temperature_sensors_-a-_transmitters/temperature_sensors/ntc10k3-n38p14-01?srsltid=AfmBOoq9_-BkfwnmWUoqQKwz0VR2Vfc8l8rtTNl9X8382e33ZclrY4tA)
Thermistor response curve [here](https://cdn.automationdirect.com/static/support/techqa/10K-3_thermistor_resistance.pdf)
Thermistor voltage divider explained [here](https://www.build-electronic-circuits.com/voltage-divider/)

### NAU7802 Notes
NAU7802 is for the load cell. It appears that the maximum excitation voltage provided by the LDO on the breakout board is 4.5 V, which is below the load cell's range of 5-10 V excitation [load cell link](https://www.amazon.com/dp/B0CPSL6KX7/ref=dp_iou_view_item?ie=UTF8&psc=1&th=1) (1000 kg, 58 mm). This should be ok, but this does introduce additional sources of error. It is also likely possible to use an external excitation supply if desired.

### Ignition Relay Notes
[Relay link](https://www.elecbee.com/en/product-detail/5v-1-channel-level-trigger-optocoupler-relay-module-high-or-low-level-trigger-relay-board-for-raspberry-pi-arduino-relay-diy_29060?utm_term=&utm_campaign=&utm_source=adwords&utm_medium=ppc&hsa_acc=9958698819&hsa_cam=23146566611&hsa_grp=195696585111&hsa_ad=796079352697&hsa_src=g&hsa_tgt=pla-2466379623765&hsa_kw=&hsa_mt=&hsa_net=adwords&hsa_ver=3&gad_source=1&gad_campaignid=23146566611&gclid=Cj0KCQjw7cLOBhDmARIsAGsuA0lqxyTn1UGvM2u_73KeJuHUsSg_l_JmbtH1cwgAZSAlX2vw9voNnAIaAuCTEALw_wcB)
Need to make sure the LED on the relay is functioning as expected (probably LED off to indicate relay off?).
Additionally, for safety reasons, it would be great to have a physical switch like [this](https://www.adafruit.com/product/3218) (we might already have one something like this) on the 12V supply to the relay as a redundant hardware switch. Maybe? Not sure this is 100% necessary but it would be good peace of mind and good practice.


# Docs
Pin assignments for Nucleo board are listed in the manual pg. ~38

# Operation Notes
### Data/Debugging/Flashing MCUs While Under External Power
The 5V/USB power connector trace has been cut for both Teensys (this separates USB and external power). This allows USB to be connected for data only, while the board is powered with external 5V at the same time.

For the Nucleo-144 board, follow instructions on pg. 22 (section 7.4.6) of the manual: connect the USB cable only after the board has been powered on via external power.


# random notes about PlatformIO and related stuff that is likely no longer relevant
To install a library in VSCode PlatformIO: Go to PlatformIO Home -> Libraries. Then search for the library and add it. This will add the library as a reference in the `lib_deps` section of the `platformio.ini` file and will add the library to the `<Project>/.pio/libdeps/<chipname>/` folder. Libraries are generally installed per-project like this unless configured otherwise. For already-included libraries (such as `Wire` or `SPI` for the Teensy), you can simply add the library definition directly to the `lib_deps` section (I think?), or maybe this is not needed and you can just do the usual `#include` entry at the top of your code. (If needed, you can directly reference the library GitHub in the `lib_deps` section instead, and this will install the library in the `.pio` folder.

Basics of PlatformIO for Teensy [here](https://forum.pjrc.com/index.php?threads/tutorial-how-to-use-platformio-visual-code-studio-for-teensy.66674/)

There is an STM32 plugin for VSCode with info [here](https://www.st.com/content/st_com/en/campaigns/stm32-vs-code-extension-z11.html)
