import numpy as np
import scipy.signal as signal

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


#Runtime weigthed moving average FIR Filter
# ---Not explored further---

#Runtime runnig average
def runnig_average(data,weight):
    out = np.zeros_like(data)
    tmp = 0
    for i in range(len(data)):
        tmp = weight*tmp + weight*data[i]
        out[i] = tmp
    return out

#Runtime 1st order lowpass filter


#2nd order Buttersworth filter
def butter_static(order,w_c,data,fs):
    butter_sos = signal.butter(order, w_c, output='sos', fs=fs)
    out = signal.sosfilt(butter_sos, data)
    return out

#Runtime 2nd order Buttersworth filter
def butter_running(order,w_c,data,fs):
    # b,a = signal.butter(order, w_c, output='ba', fs=fs)
    # out = signal.lfilter(b, a, data)
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