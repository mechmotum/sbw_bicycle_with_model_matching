# TU Delft Steer-by-Wire Bicycle
![Picture of the steer-by-wire bicycle](/steer_by_wire_bicycle)
A GitHub repository for the Model matching control research performed on the Bicycle Lab's Steer-by-Wire bicycle.
Which lead to the thesis "Model Matching Control Applied to Bicycles".
In this thesis the goal is to use model matching control to virtually alter the bicycle physical parameters.
The research showed this is possible in specific cases.
This repository contains all the code necessary to perform the experiments and analyses performed during the thesis.

## Acknoledgements
For the steer-by-wire system this code builds on Georgios Dialynas's and Simonas Drauksas's work: 
- Original repository: [gdialynas/Steer-by-wire-bicycle](https://github.com/gdialynas/Steer-by-wire-bicycle), and [gdialynas/Steer-by-wire-bicycle](https://github.com/mechmotum/TUDelft-SbW-Bicycle)
- Paper: [Dialynas, Georgios & Schwab, Arend & Happee, Riender. (2018). Design and implementation of a steer-by-wire bicycle](https://www.researchgate.net/publication/328808185_Design_and_implementation_of_a_steer-by-wire_bicycle).

## File structure
`data_analysis/` - Map containing several scripts analysis the logged data of the model matching experiments.

`docs/` - Documentation on the steer-by-wire bicycle system.

`imu_calibration` - Script that calculates the rotation matrix between the body fixed frame and the IMU frame.

`inventory/` - (Outdated) inventory of the electrical and mechanical spare parts. Copied over from Simonas's github.

`model matching gain calculation/` - Map Containing the scripts for the model matching controller design.

`sensitivity analysis/` - Map containing the scripts for the model matching controller sensitivity analysis.

`simulations/` - Map containing scripts for simulating the steer-by-wire bicycle and controller, and several theoretical analyses.

`teensy/` - Map containing the scripts that are run by the teensy controller.