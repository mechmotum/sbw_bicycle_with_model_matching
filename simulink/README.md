# Simulink

Here are the MATLAB scripts and Simulink models needed to compile and run the MPC controller.

## Requirements
1. Windows 10 computer with MATLAB 2022a (that was the version used, your mileage may vary using other versions)
2. [qpOASES v.3.2.1 with Simulink interface](https://github.com/coin-or/qpOASES)
3. [Simulink Desktop Real-Time](https://nl.mathworks.com/products/simulink-desktop-real-time.html) toolbox with the kernel installed to run the controller in real time and communicate with libsurvive and the bicycle
4. [Aerospace Blockset](https://nl.mathworks.com/products/aerospace-blockset.html) toolbox to convert quaternions to Euler angles
5. [DSP System](https://www.mathworks.com/products/dsp-system.html) toolbox to calculate the moving average

## Usage
0. Make sure all external hardware is connected: 
   - Raspberry Pi should be connected to the PC using an Ethernet cable.
   - Teensy (or other Arduino with two hardware Serial ports) acting as a Bluetooth receiver should be connected to the PC using a USB cable. 
   - Neokey TrinKey (or other microcontroller with two buttons) shuold be connected to the PC using a USB cable.
0. Place the bicycle in the middle of the treadmill, turn on the HTC Vive Tracker on the bicycle and start the `survive-udp` program on the Raspberry Pi with the `--force-calibrate` flag.
0. Run MATLAB script `main.m`.
   - It will set all the required variables for the controller.
   - The controller will asume that the first trial will be "Familiarization + Baseline 1".
   - It will open `getAngleBias.slx` Simulink model which will calculate the yaw and lean angle biases coming from HTC vive that are there due to the Tracker's positioning on the bicycle. The calibration takes 60 seconds.
   - It will open `mpcController.slx` Simulink model, which is where the controller is implemented.
0. Start the Unity game located in `../unity/SbW-game-built` which is the visualisation of the task.
0. Set the simulation time to `inf` and press "Run in Real-Time".
0. Turn on the bicycle.
0. In order to start the trial, press the capacitive button on the Neokey TrinKey (or its equivalent on another microcontroller). The gates should now start to appear in the visualisation.
0. When the trial ends, the visualisation will display `END` at the top of the screen.
0. Stop the simulation, save the `out` variable from the MATLAB workspace as this is where all the data is stored.
0. Turn off the bicycle.
0. Run the appropriate script from `./functions/` before the next trial.
0. Repeat steps 5-11 for the other trials.

## Options
The MATLAB script `main.m` loads `./mat_files/opts.mat` file which contains a structure with required options. If this file is not found, it is generated with default values using `./functions/generateOptionsFile.m` function.

The options are:
- `Ts_mpc` - the sampling time for the controller, in seconds. Default value - `1/75`
- `Ts_udp` - the sampling time for the UDP ports, in seconds. Default value - `1/320`
- `Ts_ser` - the sampling time for the serial ports, in seconds. Default value - `1/200`
- `Tinterval` - the time distance between the gates, in seconds. Default value - `6`
- `TShorizon` - the length of the MPC (Simulink) horizon, in seconds. Default value - `2`
- `TUhorizon` - the length of the visualisation (Unity) horizon, in seconds. Default value - `5`
- `Tpreview` - how soon the controller is made aware of the upcoming gate, in seconds. Default value - '4'
- `Tafter` - how long the controller should remember the location of the previous gate, in seconds. Default value - '2'
  - `Tpreview + Tafter` needs to be equal to or less than `Tinterval`
  - `Tafter` stabilises the controller as the controller will try to pass the gate in a straight line
- `blockLength` - trials are split into blocks, each of which consists of `blockLength` number of gates. Default value - `10`
- `gateNumber` - the number of different possible positions of the gates. Default value - `7`
- `maxWidth` - the furthest lateral position of the centre of the gate, relative to the centre of the treadmill, in meters. Default value - `0.2`
- `randSeed` - the seed to generate pseudo-random gate positions. Default value - `20220729`
- `trackerHeight` - the height of the HTC Vive Tracker, relative to the rear wheel contact point, in meters. Default value - `0.9`
- `steerTorqueLim` - maximum allowed torque coming from the controller, in Newton-meters. Default value - `3`
- `fCutoff` - the cutoff frequency for the low-pass filter, in Hertz. Default value - `8`
- `ub_[x,u], lb_[x,u]` - upper and lower bounds for the states and the control commands, respectively. Default values -
  - `ub_x = [0.5; inf; deg2rad(20); deg2rad(40); inf; inf]`
  - `lb_x = -ub_x`
  - `ub_u = steerTorqueLim`
  - `lb_u = -ub_u`
- `Q1` - maximum value of the MPC weight for the lateral position. Default value - `10`
- `Q2` - maximum value of the MPC weight for the yaw. Default value - `10`
- `Q3` - maximum value of the MPC weight for the lean. Default value - `3`
- `R` - MPC weight on the control command. Default value - `1`
- `Tend` - length of one block, in seconds. Calculated from `Tinterval * blockLength`
- `nSH` - number of steps in the MPC (Simulink) horizon. Calculated from `TShorizon / Ts_mpc`
- `nUH` - number of steps in the visualisation (Unity) horizon. Calculated from `TUhorizon / Ts_mpc`
- `nX` - number of states in the bicycle model. Default value - `6`
- `nU` - number of control inputs in the bicycle model. Default value - `1`

## File structure

`./functions/` - Contains MATLAB functions:
  - To calculate and generate required variables for the MPC controller (`generateOptionsFile.m`, `generateMatrices.m`, `generateReference.m`);
  - To set up other experiment trials (`readyTraining1.m`, `readyTraining2.m`, `readyRetention.m`);

`./mat_files/` - Contains `.mat` files needed by `main.m`:
  - `bike_ss.mat` contains a continuous state space model of the bicycle. By default it uses the "Rigid Rider bicycle" model. Obtained by using the scripts of [Jason K. Moore](https://github.com/moorepants/HumanControl).
  - `gate_locations.txt` file is generated by `generateReference.m` and contains the gate location data for the visualisation. Should be copied to `../unity/SbW-game-built/SbW-game_Data/StreamingAssets/`.
  - `matrices.mat` contains the required matrices for MPC/QP. Generated by `generateMatrices.m` according to the options and a discrete state space model.
  - `opts.mat` contains the options described above. Default options are generated by `generateOptionsFile.m`.
  - `structs.mat` contains the structs required by the Simulink model. Generated by `generateReference.m` according to the options.
  
 `./qp/` - Contains `qpOASES_SQProblem.[cpp, mexw64]` and `qpOASES_simulink_utils.cpp` which are required by the Simulink model to solve QP problems.

`main.m` - The main MATLAB script, responsible for setting up the MPC controller's variables, launching Simulink.

`mpcController.slx` - The MPC controller implementation as a Simulink model. It uses UDP to communicate with libsurvive and Unity, uses Serial to communicate with the steer-by-wire bicycle, and uses the qpOASES to solve the linear QP problem.

`getAngleBias.slx` - This Simulink model uses UDP to communicate with libsurvive. It runs for 60 seconds and calculates the average of the yaw and lean angles from the HTC Vive Tracker. This is needed since the Tracker is not mounted at exactly 0 yaw or lean angle compared to the bicycle and can move between participants. Therefore the calculated average is used to correct for this in `mpcController.slx`.

`README.md` - This file.
