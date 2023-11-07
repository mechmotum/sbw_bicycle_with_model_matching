import serial.tools.list_ports
import time
import numpy as np

##------[Define constants
#TODO: teensy uses native baudrate intead of defined one with Serial.begin(baudrate). Write a script selecting this native baudrate.
BAUDRATE = 9600


##------[Select correct port
# Present list of ports
ports = serial.tools.list_ports.comports()
for p in ports:
    print(p)

# Chose correct one
teensy_port = "COM" + input("Select serial port: COM")

# Check if it exists
if not sum(1 for dummy in serial.tools.list_ports.grep(teensy_port)): #check if the generator(/iterable) returned is empty
    print("Port name not valid, please select the correct COM")
    print("Press Ctrl+C to exit program")
    while(1):
        pass


##------[Create communication object
''' 
Note: make sure that you have the following at the start of your setup:
Serial.begin(BAUDRATE)
while(!Serial){}
This forces the micro controller to waint untill the COM port is opened.
Either by the serial monitor, or by this code.
'''
teensy_serial = serial.Serial(
    port=teensy_port,
    baudrate=BAUDRATE
    )

#-------[Reconnect to comport
teensy_serial.close()
teensy_serial.open()


#-------[Create data to send
output = np.array([1.00025], dtype=np.float32)


##------Communicate
while True:
    time.sleep(0.1)
    teensy_serial.write(output.tobytes())
    if teensy_serial.in_waiting:
        input_b = teensy_serial.read_until(b'\r\n').rstrip(b'\r\n')
        output = np.frombuffer(input_b, dtype=np.float32)


# SITES
# https://stackoverflow.com/questions/58813707/converting-python-float-to-bytes
# https://numpy.org/doc/stable/reference/generated/numpy.frombuffer.html
# https://stackoverflow.com/questions/8216088/how-to-check-the-size-of-a-float-in-python

# time.sleep(1)
# bytes_out = teensy_serial.write(output.tobytes())
# print(bytes_out)
# print(np.frombuffer(output.tobytes(), dtype=np.float32))
# if teensy_serial.in_waiting:
#     bytes_in = teensy_serial.in_waiting
#     print(bytes_in)
#     input_b = teensy_serial.readline().rstrip(b'\n\r')
#     print(input.decode('utf-8').rstrip('\n\r'))
#     print(input_b)
#     output = np.frombuffer(input_b, dtype=np.float32)
#     print(output)
