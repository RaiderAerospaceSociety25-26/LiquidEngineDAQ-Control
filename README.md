# LiquidEngineDAQ-Control
Repository for DAQ and control system for the liquid engine and test stand being developed by the Raider Aerospace Society (RAS) PigeonWorks Liquid Engine Team during the 2025-2026 academic year.

# Firmware
Basics of PlatformIO for Teensy [here](https://forum.pjrc.com/index.php?threads/tutorial-how-to-use-platformio-visual-code-studio-for-teensy.66674/)

Planning to use PlatformIO for both Teensy and Nucleo144 board.

There is an STM32 plugin for VSCode with info [here](https://www.st.com/content/st_com/en/campaigns/stm32-vs-code-extension-z11.html)

The radios work via Meshtastic (only using TX/RX pins on the boards as far as I know)

# Hardware
### ADS1256 Notes
ADS1256 analog signal input maxes out at AVDD minus 2V (so 3V maximum analog signal) if using self-calibration routines with the buffer. If we have the capability to perform calibration and write calibration values directly to the register, we could bypass this and use the full range (analog signals accepted up to AVDD), but this would likely be fairly difficult. HOWEVER, it looks like the buffer is the issue. The buffer increases input impedance to lower noise/settling time/do other stuff I don't fully understand. Overall, the buffer is needed for high-output impedance sensors. With our pressure transducers, we have a 0.5V-4.5V signal. Generally, pressure transducers are relatively low impedance (mA PTs are very low impedance, mV PTs are higher impedance I think). Briefly looking up PTs, it appears that their impedances are on the order of 100 Ω, which should have no issue as the input impedance on the ADS1256, even with the buffer off, is (150 kΩ / PGA) for PGA values of 16 or lower (and we won't need a high PGA value).  Therefore, I don't think the buffer is essential. So we're good, we just have to make sure that the buffer stays off if we're measuring pressures higher than ~60%-70% of the range of the pressure transducer. Note that the ADC will require a lot more current with the buffer off. Note that the buffer is actually off by default too. Also make sure to keep the PGA low (which won't be an issue because the signal we're reading is already 0.5V-4.5V).

# Docs
Pin assignments for Nucleo board are listed in the manual pg. ~38

# Operation Notes
### Data/Debugging/Flashing MCUs While Under External Power
The 5V/USB power connector trace has been cut for both Teensys (this separates USB and external power). This allows USB to be connected for data only, while the board is powered with external 5V at the same time.

For the Nucleo-144 board, follow instructions on pg. 22 (section 7.4.6) of the manual: connect the USB cable only after the board has been powered on via external power.
