# NeoKey

I am using a NeoKey Trinkey microcontroller with a mechanical key switch and a capacitive touch sensor to control the trial's start and end during the experiment. The board also has a programmable RGB LED. The board is programmed using CircuitPython.

The mechanical key switch acts as a "Emergency STOP" button for the MPC controller. When the button is pressed down, it starts glowing red and Simulink sets the MPC control torques to 0, no matter what torque controller calculates.

The capacitive touch is used to start/stop the trial. When the sensor is touched, it starts glowing green and Simulink starts the trial.

This is achieved through Serial communication between NeoKey and Simulink. By default, NeoKey sends out a string containing two integers separated by a comma. The first integer describes the state of the capacitive touch, and the second integer describes the state of the button.

The state of the button can take two values - 1 or 2. 1 stands for "Button was not pressed, red glow OFF", while 2 stands for "Button was pressed, red glow ON".

The state of the capacitive touch can take two values - 3 and 4. 3 stands for "Capacitive touch was not touched, green glow OFF". 4 stands for "Capacitive touch was touched, green glow ON".

The red glow will always overwrite the green glow.
