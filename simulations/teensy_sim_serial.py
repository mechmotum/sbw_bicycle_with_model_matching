import serial.tools.list_ports
import numpy as np

class TeensySimSerial:
    def __init__(self, baud_rate):
        #------[Initialize variables
        teensy_port = self.select_port()
        self.baud_rate = baud_rate
       
        #------[Create communication object
        ''' 
        Note: make sure that you have the following at the start of your teensy
        code setup:
        Serial.begin(BAUDRATE)
        while(!Serial){}
        This forces the micro controller to waint untill the COM port is opened.
        Either by the serial monitor, or by this code.
        '''
        self.com = serial.Serial(
            port=teensy_port,
            baudrate=self.baud_rate
            )
        return
    
    def select_port(self):
        '''
        Select correct port with help of user.
        '''
        # Present list of ports
        ports = serial.tools.list_ports.comports()
        for p in ports:
            print(p)

        # Chose correct one
        teensy_port = "COM" + input("Select serial port: COM")

        # Check if it exists
        if not sum(1 for dummy in serial.tools.list_ports.grep(teensy_port)): #check if the generator(/iterable) returned is empty
            print("\n................")
            print("Port name not valid, please select the correct COM")
            print("Press Ctrl+C to exit program")
            while(1):
                pass
        return teensy_port

    def sim_tx(self,meas,data_type):
        '''
        Send the measurement data calculated by the simulation 
        to the teensy.
        Make sure the data expected by the teensy is also 'dtype'.
        '''
        meas = np.array(meas, dtype=data_type) # Ensure the data is a 'data_type' array.
        # print(meas, meas.tobytes(), np.frombuffer(meas.tobytes(), dtype=data_type)) # for debugging
        return self.com.write(meas.tobytes()) # Convert to bytes and send

    def sim_rx(self, data_type):
        '''
        read the commands calculated by the controller.
        Note: Check what kind of endline command the hardware sends.
        For teensy (and most likely arduino) it is b'\\r\\n'.
        Make sure the data sent by the teensy is indeed 'data_type'.
        ''' #Should be b'\r\n' (escapes are used for vscode documentation features purposes.)
        input_b = self.com.read_until(b'\r\n').rstrip(b'\r\n')
        # print("input in bytes: ", input_b) # for debugging
        input = np.frombuffer(input_b, dtype=data_type,)
        return input

    def reconnect(self):
        '''
        Close and open the the serial port in order
        to make sure you are connected to it.
        '''
        self.com.close()
        self.com.open()
        return
    
    def in_waiting(self):
        '''
        Wrapper for the similar named Serial class property 'in_waiting'
        returns the number of bytes currently in the input buffer
        '''
        return self.com.in_waiting

##-----------------------------EXAMPLE-----------------------------##
# import time

# ##------[Define constants
# #TODO: teensy uses native baudrate intead of defined one with Serial.begin(baudrate). Write a script selecting this native baudrate.
# BAUDRATE = 9600
# COM_DATA_TYPE = np.float32

# #-------[create serial
# teensy_serial = TeensySimSerial(BAUDRATE,COM_DATA_TYPE)

# #-------[Reconnect to comport
# teensy_serial.reconnect()

# #-------[Create data to send
# output = np.array([123, 12.5, 15.5, 4846, 45354.1, 465, -1], dtype=np.float32)


# ##------Communicate
# while True:
#     time.sleep(1)
#     teensy_serial.sim_tx(output)
#     if teensy_serial.in_waiting():
#         input = teensy_serial.sim_rx()
#         print(input)
#         output = (input[0]/input[1])*np.ones((7,), dtype=np.float32) #Be aware that every array you make has to specify that its numbers are float32



##-----------------------------SITES-----------------------------##
# https://stackoverflow.com/questions/58813707/converting-python-float-to-bytes
# https://numpy.org/doc/stable/reference/generated/numpy.frombuffer.html
# https://stackoverflow.com/questions/8216088/how-to-check-the-size-of-a-float-in-python


##-----------------------------DEBUG HELP-----------------------------##
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
