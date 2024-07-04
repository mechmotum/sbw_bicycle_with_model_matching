'''
Code to see what the byte sizes are for all values send to 
and from the teensy.
'''
import numpy as np

print(np.iinfo(np.int8).bits)
print(np.iinfo(np.uint16).bits)
print(np.iinfo(np.int32).bits)
print(np.finfo(np.float32).bits)