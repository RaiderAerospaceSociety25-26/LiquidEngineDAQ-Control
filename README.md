### LiquidEngineDAQ-Control
Repository for DAQ and control system for the liquid engine and test stand being developed by the Raider Aerospace Society (RAS) PigeonWorks Liquid Engine Team during the 2025-2026 academic year.

### Firmware
Basics of PlatformIO for Teensy [here](https://forum.pjrc.com/index.php?threads/tutorial-how-to-use-platformio-visual-code-studio-for-teensy.66674/)
Planning to use PlatformIO for both Teensy and Nucleo144 board.
There is an STM32 plugin for VSCode with info [here](https://www.st.com/content/st_com/en/campaigns/stm32-vs-code-extension-z11.html)
The radios work via Meshtastic (only using TX/RX pins on the boards as far as I know)

### Hardware
ADS1256 analog signal input must be held at AVDD minus 2V (so 3V maximum analog signal) if using self-calibration routines. If we have the capability to perform calibration and write calibration values directly to the register, we could bypass this and use the full range (analog signals accepted up to AVDD), but this would likely be fairly difficult.

### Docs
Pin assignments for Nucleo board are listed in the manual pg. ~38

### Operation Notes
The 5V/USB power connector trace has been cut for both Teensys (this separates USB and external power). This allows USB to be connected for data only, while the board is powered with external 5V at the same time.
For the Nucleo-144 board, follow instructions on pg. 22 (section 7.4.6) of the manual: connect the USB cable only after the board has been powered on via external power.
