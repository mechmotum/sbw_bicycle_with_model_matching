# Teensy

This folder contains the software for the Teensy 4.1 installed in the rear electronics box.
The Teensy is responsible for running the PD controller that makes sure that the handlebar
and fork angles match up.

## File structure
`lib/` - This folder contains the required external libraries. Currently, you can find Bolderflight's MPU9250 library and its dependencies, as well as RF24 for the radio communication, and SdFat for the SD card logging. Libraries like `Arduino`, `Encoder`, `SPI`, `RingBuf` are either standard Teensy 4.1 libraries, standard PlatformIO libraries, or included with previously mentioned libraries.

`src/` - This folder contains the main code base. Currently, everything is contained in the `main.cpp` file.

`log/` -  Conaining the log files from several experiments. (Except for the log files used in the scripts them selves (e.g. `log/bodetest-4m_per_s/`, `log/eigen_[XXX]_sil6.5n2_[XXX]kph.log`, etc) These should actually not be on the github, so you can delete them. I did not have the time to figure out which files are still used.)

`.gitignore` - This file excludes the `.pio` folder (used for temporary compilation files) and `.vscode` folder (used for my VSCode configuration).

`platformio.ini` - This is a PlatformIO project configuration file. Currently settings are to use the Arduino framework for the Teensy 4.1 board.