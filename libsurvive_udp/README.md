# Libsurvive UDP

This project uses an HTC Vive Tracker 3.0 to localise and track the bicycle's position on the treadmill, as well as obtaining lean angle and rate, and yaw angle. To do so, we need some way to interface with the Tracker and read its data.

One option is to use SteamVR (OpenVR is just renamed SteamVR), but that requires the use of a game engine, such as Unity. In order to use HTC Vive Trackers in Unity, we also need to have an HMD connected (or use workarounds which are not officially supported). There are other downsides to using Unity, such as a limited number of Trackers that can be used at once, or a lower sampling rate than the hardware allows for.

Another option is to use [libsurvive](https://github.com/cntools/libsurvive) -- a community-built open-source tool, that was developed with the goal of tracking HTC Vive Trackers without the need for an HMD.

Neither Unity, nor *libsurvive can nativey communicate with Simulink. To do so, the code hosted here was written. It uses libsurvive's Simple API to read the data from the Trackers and sends it through UDP to Simulink. The data sent contains the Position of the tracker, the Axis-Angle Rotation rates, and the Orientation of the tracker (in the form of a quaternion). 

To implement this, libsurvive's `api_example.c` was used as foundation. The UDP part of the code was written by following Beej's amazing ["Guide to Network Programming"](https://beej.us/guide/bgnet/).

## Building

This code only works on Linux machines due to UDP requiring different implementation on Windows. MacOS should work as well, but I have not tried it. Code was tested on Pop!_OS 22.04 (based on Ubuntu 22.04) and Raspberry Pi OS Lite 32-bit (based on Debian 11 Bullseye).

Instructions (for Ubuntu/Debian):
1. Clone the libsurvive's github repository `git clone git@github.com:cntools/libsurvive.git --recursive`
2. Install *udev* rules so that you can interface with the HTC Vive Tracker dongle `cd libsurvive && sudo cp ./useful_files/81-vive.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger`
3. Install required packages for building `sudo apt install build-essential zlib1g-dev libx11-dev libusb-1.0-0-dev freeglut3-dev liblapacke-dev libopenblas-dev libatlas-base-dev cmake`
4. Copy `survive-udp.c` and `CMakeLists.txt` from this repository to the same folder as libsurvive
5. Build using `make`
6. Now you can run `./bin/survive-udp`! The output in the console will show the X-Y-Z position of the tracker

## Customization

In my project, I used a Raspberry Pi 4B with 4GB of RAM to run `survive-udp`, therefore it is acting as a server. The RasPi was connected to a Desktop computer running Windows 10 and Simulink (the client) through the Ethernet cable (you might need to tinker around with Windows settings to make Windows see the Ethernet cable as LAN, rather than a bad internet connection). Port 4950 was used for UDP communication (don't forget to allow this port through Window's firewall or you won't get any data!). 

If you want to change the port used or adjust the IP address of the client, you should change the defined values of `SERVERPORT` and `SENDTOIP` in `survive-udp.c`. To adjust what exactly is being sent out through UDP, modify the `sprintf` command on line 85. Currently, the data is sent out each time libsurvive calculates a new pose (the case of `SurviveSimpleEventType_PoseUpdateEvent`). This can be adjusted to, for example, send data after a button was pressed (using the case `SurviveSimpleEventType_ButtonEvent`). After making your changes, run `make` again to build a new `survive-udp` binary. 

## File structure
`CMakeLists.txt` - The only difference between this file and the similarly-named file on libsurvive's repository is the addition of `survive-udp` to `SURVIVE_EXECUTABLES` on line 202.

`survive-udp.c` - C code that uses libsurvive's Simple API to read the data from the tracker and send it out through UDP.