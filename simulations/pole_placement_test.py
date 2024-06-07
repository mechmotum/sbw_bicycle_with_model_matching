import numpy as np
import matplotlib.pyplot as plt
import scipy.signal as sign

## Setup Matrices
 # Dimensions
n = 2
m = 1
p = n

 # Reference System
S_r = np.matrix([[1,2],[1,1]])
D_r = np.matrix([[-0.1,0],[0,-0.6]])
T_r = np.linalg.inv(S_r)
A_r = S_r*D_r*T_r

 # Uncontrolled system
# A_n = np.matrix([[2,-4],[0.2,-3]])
# B_n = np.matrix([[1,2],[2,3]])
A_n = np.matrix([[0.9,0],[2.5,-1.1]])
B_n = np.matrix([[1],[1.5]])
print(f"\n(A_n,B_n) controllable: {n==np.linalg.matrix_rank(np.hstack((B_n,A_n*B_n)))}")
print(f"\nEigenvalues uncontrolled: {np.linalg.eigvals(A_n)}\n")
 
 # Controlled System
F = sign.place_poles(A_n,B_n,poles=[-0.1,-0.6])
A_c = (A_n - B_n*F.gain_matrix)

# F_mm = np.linalg.inv(B_n)*(A_r - A_n)
# A_mm = (A_n + B_n*F_mm)
F_mm = (A_r[0] - A_n[0])
A_mm = (A_n + B_n*F_mm)

print(f"Eigenvalues controlled (pp): {np.linalg.eigvals(A_c)}")
print(f"Eigenvectors controlled (pp):\n{np.linalg.eig(A_c).eigenvectors}")
print(f"Eigenvalues controlled (mm): {np.linalg.eigvals(A_mm)}")
print(f"Eigenvectors controlled (mm):\n{np.linalg.eig(A_mm).eigenvectors}")
print(f"Eigenvalues reference: {np.linalg.eigvals(A_r)}")
print(f"Eigenvectors reference:\n{np.linalg.eig(A_r).eigenvectors}")

print(f"\nA matrix controlled (pp):\n {A_c}")
print(f"\nA matrix controlled (mm):\n {A_mm}")
print(f"\nA matrix reference:\n {A_r}")

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
T_mm,y_mm,x_mm = sign.lsim((A_mm,B,C,D),u_vec,time,x0,interp=True)


## Plot
plt.figure(dpi=110)
plt.title("Toy example: model matching with pole placement",fontsize=24)
plt.ylabel("state value [-]",fontsize=16)
plt.xlabel("time [s]",fontsize=16)
plt.plot(T_r,x_r,linestyle='-',label=["reference 1", "reference 2"])
plt.plot(T_c,x_c,linestyle='--',label=["controlled (pp) 1", "controlled (pp) 2"])
plt.plot(T_mm,x_mm,linestyle=':',label=["controlled (mm) 1", "controlled (mm) 2"])
plt.grid()
plt.legend(fontsize=12)
plt.show()