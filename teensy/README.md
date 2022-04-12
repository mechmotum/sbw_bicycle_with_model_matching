# Teensy

This folder contains the software for the Teensy 4.1 installed in the rear electronics box.
The Teensy is responsible for running the PD controller that makes sure that the handlebar
and fork angles match up.

## File structure
`lib/` - This folder contains the required external libraries. Currently only Bolderflight's MPU9250 library and its dependencies are included here. Other libraries (`Encoder`, `SPI`, `SD`, `TeensyThreads`) are standard Teensy 4.1 libraries and come in-built.

`src/` - This folder contains the main code base. Currently, everything is contained in the `main.cpp` file.
