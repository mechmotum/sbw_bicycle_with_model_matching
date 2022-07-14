# Simulink

Here are the MATLAB scripts and Simulink models needed to compile and run the MPC controller.

## Requirements
1. Windows 10 computer with MATLAB 2022a (that was the version used, your mileage may vary using other versions)
2. [qpOASES v.3.2.1 with Simulink interface](https://github.com/coin-or/qpOASES)
3. [Simulink Desktop Real-Time](https://nl.mathworks.com/products/simulink-desktop-real-time.html) toolbox with the kernel installed to run the controller in real time and communicate with libsurvive and the bicycle
4. [Aerospace Blockset](https://nl.mathworks.com/products/aerospace-blockset.html) toolbox to convert quaternions to Euler angles
5. [DSP System](https://www.mathworks.com/products/dsp-system.html) toolbox to calculate the moving average

## Usage
1. Run MATLAB script `controllerInit.m`, which is going to set all the required variables.
2. At the end, a Simulink model `controllerModel.slx` will open. 
3. (If using Unity) Running the MATLAB script will also generate a `reference_strings.txt` file, which should be copied to Unity's project (`/unity/SbW-game/Assets/StreamingAssets/` if running the game from the editor, or `/unity/SbW-game-built/SbW-game_Data/StreamingAssets/` if running the game standalone),  as it contains the data for the reference line visualization and lets the rider see the same reference the controller sees.
4. (If using HTC Vive Tracker) Start the `libsurvive-udp` program on the Raspberry Pi (or other machine that communicates with the Tracker).
5. Plug the Teensy acting as a Bluetooth receiver into the PC.
6. Set the required length of the simulation and run the model. The outputs will be saved in the `out` structure in MATLAB's workspace.
7. Turn on the Steer-by-Wire bicycle. The controller's inputs are only sent to the motors if the switch on the handlebars is on its right-most position.

## Options
The main MATLAB script `controllerInit.m` has a couple of options for the user.

The user can adjust some simple options, such as:
- `Ts_mpc` - the sampling time for the controller, in seconds.
- `Ts_udp` - the sampling time for the UDP ports, in seconds.
- `Ts_ser` - the sampling time for the serial ports, in seconds.
- `Thorizon` - the length of the MPC horizon, in seconds.
- `tracker_height` - the height of the HTC Vive Tracker, relative to the rear wheel contact point.
- `steerTorqLim` - maximum allowed torque coming from the controller.
- `ub_[x,u], lb_[x,u]` - upper and lower bounds for the states and the control commands.
- `Q, R` - weight matrices for the states and control commands.
- `width` - the width of the lane change, in meters.
- `slope` - the slope of the lane change, in seconds. Bigger number means a more aggressive lane-change.
- `v` - the speed of the bicycle, in meters/second. Assumed to be constant throughout the experiment.
- `Tend` - time for the end of the `t_ref` field in the generated reference trajectory, can be used to generate a single lane-change instead of a double lane-change.

## File structure

`controllerInit.m` - The main MATLAB script, responsible for setting up the MPC controller's variables, generating a reference trajectory.

`controllerModel.slx` - The Simulink model. It uses UDP to communicate with libsurvive and Unity, uses Serial to communicate with the steer-by-wire bicycle, and uses the qpOASES to solve the linear QP problem.

`generateReference.m` - This MATLAB function generates the reference trajectory for the double lane-change manoeuvre. The function takes in 5 parameters: `tEnd`, `tSample`, `tChange`, `speed`, and `lWidth`. The function returns an array with the first column being time `t_ref`, second column is the coordinate in the longitudinal direction `x_ref`, and the third column is the coordinate in the lateral direction `y_ref`. The first lane-change starts after 5 seconds, the second one starts 10 seconds after the first one ends. The `tChange` parameter is used to set the aggressiveness of the lane-change: the bigger the `tChange` is, the slower the lane change will be. This function can also generate a single lane-change reference if `tEnd` is shorter than `15 + tChange`. The parameter `lWidth` adjusts how big of the lane change will be and only affects `y_ref`, while the parameter `speed` only affects `x_ref`.

`qpOASES_QProblem.[cpp, mexw64], qpOASES_simulink_utils.cpp` - Files required to solve the QP using qpOASES.

`README.md` - This file.

`rigidRiderStateSpace.mat` - File containing the state space model of the Rigid Rider bicycle. Obtained by using the scripts of [Jason K. Moore](https://github.com/moorepants/HumanControl).