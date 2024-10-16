# Simulations
This map contains all scripts used to perform simulations of the bicycle. There are two types of simulation, hardware in the loop simulation and computer simulation. In the hardware in the loop simulation the microcontroller (teensy) is connected to the computer. The computer acts as if it is the bicycle, simulating the dynamics and sensor readings.
In the computer simulation, the bicycle and control are both simulated on the PC.

## Main files
Each file has a little discription of what its function is at the start of the file.
Most files in this map only contain functions used in the main scripts.
These files are the scripts that should be run:
* `check_byte_size.py`
* `pole_placement_test.py`
* `simulate_drift_data.py`
* `simulation_main.py`