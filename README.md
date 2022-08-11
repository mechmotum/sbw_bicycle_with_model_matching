# TU Delft Steer-by-Wire bicycle
A GitHub repository for the Bicycle Lab's Steer-by-Wire bicycle.

Building on Georgios Dialynas' work: 
- Original repository: [gdialynas/Steer-by-wire-bicycle](https://github.com/gdialynas/Steer-by-wire-bicycle).
- Paper: [Dialynas, Georgios & Schwab, Arend & Happee, Riender. (2018). Design and implementation of a steer-by-wire bicycle](https://www.researchgate.net/publication/328808185_Design_and_implementation_of_a_steer-by-wire_bicycle).

## File structure
`bluetooth_teensy/` - A Teensy 4.1 together with an HC-05 Bluetooth module is used as a Bluetooth interface to Simulink. This folder hosts the code for the Teensy. Written in C++ using PlatformIO.

`docs/` - Hosts the documentation for the bicycle, hardware and software.

`inventory/` - Hosts the inventory of the spare parts for the hardware.

`libsurvive-udp/` - Hosts the files to enable HTC Vive Tracker communication through UDP.

`neokey/` - A NeoKey Trinkey is used to control the trial's start and end. Code for the NeoKey is hosted here. Written in CircuitPython.

`simulink/` - Hosts the MATLAB scripts and Simulink model for the MPC controller.

`teensy/` - Contains the Teensy code that acts as the brain of the bicycle. Written in C++ using PlatformIO.

`unity/SbW-game/` - Contains the visualisation of the reference line that needs to be tracked. Developed in Unity.

`unity/SbW-game-built/` - Contains the binaries built from `unity/SbW-game/`. Built for Windows 64-bit.
