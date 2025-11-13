import numpy as np
import matplotlib.pyplot as plt

L1 = 18
L2 = 15.5
L3 = 7.5
def inverse_kenematics(x, y ,z):
    xa = (((x**2) + (y**2))**0.5)#* -1
    c = ((xa**2) + (z**2))**0.5
    # Hitung posisi dan tampilkan
    t0 = np.arctan(y/x)
    t1 = (np.arctan(xa/z))+(np.arccos(((L1**2)+(c**2)-(L2**2))/(2*L1*c)))
    t2 = np.arccos(((L1**2) + (L2**2) - (c**2))/(2*L1*L2))
    t3 = (3.14-(np.arccos(((L1**2)+(c**2)-(L2**2))/(2*L1*c)))-1.57)+(3.14-(np.arctan(xa/z))-t2)
    s0 = t0 + 3.14 -0.523
    s1 = (1.57 + (1.57-t1)) -0.523
    s2 = 1.57 - t2 + 4.71 -0.523
    s3 = 1.57 - t3 + 4.71 - 0.523
    deg0 = (s0*180)/np.pi
    deg1 = (s1*180)/np.pi
    deg2 = (s2*180)/np.pi
    deg3 = (s3*180)/np.pi
    d0 = ((deg0/300)*1024)
    d1 = ((deg1/300)*1024)
    d2 = ((deg2/300)*1024)
    d3 = ((deg3/300)*1024)
    print("theta0",(t0/3.14)*180)
    print("theta1",180-((t1/3.14)* 180))
    print("theta2",((t2/3.14)*180))
    print("theta3",(180 - ((t3/3.14)*180)))
    return
    
if __name__ == "__main__":
    # Input koordinat dari pengguna
    x = float(input("Masukkan koordinat x (cm): "))
    y = float(input("Masukkan koordinat y (cm): "))
    z = float(input("Masukkan koordinat z (cm): "))
    inverse_kenematics(x,y,z)