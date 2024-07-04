# Teensy

This folder contains the software for the Teensy 4.1 installed in the rear electronics box.
The Teensy is responsible for running the PD controller that makes sure that the handlebar
and fork angles match up.

## File structure
`lib/` - This folder contains the required external libraries. Currently, you can find Bolderflight's MPU9250 library and its dependencies, as well as RF24 for the radio communication, and SdFat for the SD card logging. Libraries like `Arduino`, `Encoder`, `SPI`, RingBuf` are either standard Teensy 4.1 libraries, standard PlatformIO libraries, or included with previously mentioned libraries.

`src/` - This folder contains the main code base. Currently, everything is contained in the `main.cpp` file.

`.gitignore` - This file excludes the `.pio` folder (used for temporary compilation files) and `.vscode` folder (used for my VSCode onfiguration).

`platformio.ini` - This is a PlatformIO project configuration file. Currently settings are to use the Arduino framework for the Teensy 4.1 board.