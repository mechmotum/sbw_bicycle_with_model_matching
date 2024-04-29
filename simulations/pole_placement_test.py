import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sign

## Setup Matrices
 # Dimensions
n = 2
m = 1
p = n

 # Reference System
S_r = np.matrix([[1,0],[0,1]])
D_r = np.matrix([[-0.1,0],[0,-0.6]])
T_r = np.matrix([[1,0],[0,1]])
A_r = S_r*D_r*T_r

 # Uncontrolled system
A_n = np.matrix([[2,-4],[0.2,-3]])
B_n = np.matrix([[1],[2]])
print(f"\n(A_n,B_n) controllable: {n==np.linalg.matrix_rank(np.hstack((B_n,A_n*B_n)))}")
print(f"\nEigenvalues uncontrolled: {np.linalg.eigvals(A_n)}")
 
 # Controlled System
F = sign.place_poles(A_n,B_n,poles=[-0.6,-0.1])
A_c = (A_n - B_n*F.gain_matrix)
print(f"Eigenvalues controlled: {np.linalg.eigvals(A_c)}")
print(f"Eigenvalues reference: {np.linalg.eigvals(A_r)}")


print(f"\nA matrix reference:\n {A_r}")
print(f"A matrix controlled:\n {A_c}")

## Simulate
 # Setup
B = np.zeros((n,m))
C = np.zeros((p,n))
D = np.zeros((p,m))

x0 = np.matrix([2,1])
time = np.arange(start=0,stop=50,step=0.01)
u_vec = np.zeros_like(time)

T_r,y_r,x_r = sign.lsim((A_r,B,C,D),u_vec,time,x0,interp=True)
T_c,y_c,x_c = sign.lsim((A_c,B,C,D),u_vec,time,x0,interp=True)


## Plot
plt.figure(dpi=110)
plt.title("Toy example: model matching with pole placement",fontsize=24)
plt.ylabel("state value [-]",fontsize=16)
plt.xlabel("time [s]",fontsize=16)
plt.plot(T_r,x_r,linestyle='-',label=["reference 1", "reference 2"])
plt.plot(T_c,x_c,linestyle='--',label=["controlled 1", "controlled 2"])
plt.grid()
plt.legend(fontsize=12)
plt.show()