'''
___[ simulated_runtime_filter.py ]___
I use filters in the teensy code. This script
tests out how several filters work when 
filtering the measurements in real time. I 
also coded filters that take the entire signal
as input for comarison.
'''

import numpy as np
import scipy.signal as signal

# For runtime filters look into FIR and IIR filters.
#    See: https://www.youtube.com/@youngmoo-kim for videos on digital filtering


#Runtime moving average FIR Filter
def mov_average(data,order):
    out = np.zeros_like(data)
    tmp = 0
    for i in range(len(data)):
        if (i >= order):
            tmp = tmp + data[i] - data[i-order]
        else:
            tmp = tmp + data[i]
        out[i] = tmp/order
    return out

#Runtime runnig average
def runnig_average(data,weight):
    out = np.zeros_like(data)
    tmp = 0
    for i in range(len(data)):
        tmp = (1-weight)*tmp + weight*data[i]
        out[i] = tmp
    return out

#Runtime 1st order lowpass filter
#  G(s) = Wc/(s+Wc) --> a = [1 Wc], b = Wc
#  Uses bilinear transformation, see: 
#   > https://www.youtube.com/watch?v=HJ-C4Incgpw
#   > https://en.wikipedia.org/wiki/Bilinear_transform
def first_order_lp(w_c,data,fs):
    a_c = [1, 2*np.pi*w_c]
    b_c = [2*np.pi*w_c]
    out = np.zeros_like(data)
    b,a = signal.bilinear(b_c, a_c, fs=fs)
    for i in range(max(len(a),len(b)),len(data)):
        tmp_a = 0
        tmp_b = 0
        for l in range(1,len(a)):
            tmp_a = tmp_a + a[l]*out[i-l]
        for k in range(len(b)):
            tmp_b = tmp_b + b[k]*data[i-k]
        out[i] = (tmp_b - tmp_a)/a[0]
    return out

#Runtime 1st order highpass filter
# G(s) = s/(s+Wc) --> a = [1 Wc], b = [1 0]
def first_order_hp(w_c,data,fs,showCoefs=False):
    a_c = [1, 2*np.pi*w_c]
    b_c = [1, 0]
    out = np.zeros_like(data)
    b,a = signal.bilinear(b_c, a_c, fs=fs)
    if(showCoefs):
        print(f"b_coefficients:\t{b}\na_coefficients:\t{a}")

    for i in range(max(len(a),len(b)),len(data)):
        tmp_a = 0
        tmp_b = 0
        for l in range(1,len(a)):
            tmp_a = tmp_a + a[l]*out[i-l]
        for k in range(len(b)):
            tmp_b = tmp_b + b[k]*data[i-k]
        out[i] = (tmp_b - tmp_a)/a[0]
    return out

#Runtime 2nd order Buttersworth filter
def butter_running(order,w_c,data,fs):
    out = np.zeros_like(data)
    b,a = signal.butter(order, w_c, output='ba', fs=fs)
    for i in range(max(len(a),len(b)),len(data)):
        tmp_a = 0
        tmp_b = 0
        for l in range(1,len(a)):
            tmp_a = tmp_a + a[l]*out[i-l]
        for k in range(len(b)):
            tmp_b = tmp_b + b[k]*data[i-k]
        out[i] = (tmp_b - tmp_a)/a[0]
    return out

## FULL SIGNAL
# 2nd order Buttersworth filter
def butter_static(order,w_c,data,fs):
    butter_sos = signal.butter(order, w_c, output='sos', fs=fs)
    out = signal.sosfilt(butter_sos, data)
    return out