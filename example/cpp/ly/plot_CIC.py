import numpy as np
import matplotlib.pyplot as plt
import math

gap = 10
pi = math.pi
f0 = np.zeros(1001)
f1 = np.zeros(1001)
f2 = np.zeros(1001)
f3 = np.zeros(1001)
f4 = np.zeros(1001)
f5 = np.zeros(1001)
f6 = np.zeros(1001)

for i in range(1001):
    f0[i] = 0
    f1[i] = 1
    f2[i] = 2
    f3[i] = 3
    f4[i] = 4
    f5[i] = 5
    f6[i] = 6


qd0 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd0.txt')[0::gap]
qd1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd1.txt')[0::gap]
qd1 = qd1 + pi / 6
qd2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd2.txt')[0::gap]
qd3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd3.txt')[0::gap]
qd3 = qd3 + pi / 3
qd4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd4.txt')[0::gap]
qd5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd5.txt')[0::gap]
qd5 = qd5 + pi / 2
qd6 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qd6.txt')[0::gap]

# qr0 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr0.txt')[0::gap]
# qr1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr1.txt')[0::gap]
# qr2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr2.txt')[0::gap]
# qr3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr3.txt')[0::gap]
# qr4 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr4.txt')[0::gap]
# qr5 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr5.txt')[0::gap]
# qr6 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/qr6.txt')[0::gap]

q = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/q.txt')[0::gap,:]
dq = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/dq.txt')[0::gap,:]
qr = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/qr.txt')[0::gap,:]
z1 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/z1.txt')[0::gap,:]
z2 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/z2.txt')[0::gap,:]
z3 = np.loadtxt('/home/robot/robot/roake_param_identify/build/ly_data/CIC/PD/z3.txt')[0::gap,:]



qr0 = qr[:,0]
qr1 = qr[:,1] + pi / 6
qr2 = qr[:,2]
qr3 = qr[:,3] + pi / 3
qr4 = qr[:,4]
qr5 = qr[:,5] + pi / 2
qr6 = qr[:,6]

q0 = q[:,0]
q1 = q[:,1]
q2 = q[:,2]
q3 = q[:,3] 
q4 = q[:,4]
q5 = q[:,5]
q6 = q[:,6]

dq0 = dq[:,0]
dq1 = dq[:,1]
dq2 = dq[:,2]
dq3 = dq[:,3] 
dq4 = dq[:,4]
dq5 = dq[:,5]
dq6 = dq[:,6]

hat_q0 = z1[:,0]
hat_q1 = z1[:,1]
hat_q2 = z1[:,2]
hat_q3 = z1[:,3]
hat_q4 = z1[:,4]
hat_q5 = z1[:,5]
hat_q6 = z1[:,6]

hat_dq0 = z2[:,0]
hat_dq1 = z2[:,1]
hat_dq2 = z2[:,2]
hat_dq3 = z2[:,3] 
hat_dq4 = z2[:,4]
hat_dq5 = z2[:,5]
hat_dq6 = z2[:,6]

hat_f0 = z3[:,0]
hat_f1 = z3[:,1]
hat_f2 = z3[:,2]
hat_f3 = z3[:,3]
hat_f4 = z3[:,4]
hat_f5 = z3[:,5]
hat_f6 = z3[:,6]

plt.subplot(4,2,1)
plt.plot(qr0, color=(26/255, 111/255, 223/255),label='$qr0$')
plt.plot(qd0, c='gray',label='$qd0$')
plt.plot(q0, color=(241/255, 64/255, 64/255),label='$q0$')
plt.plot(hat_q0, c='green',label='$\hat q_0$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(qr1, color=(26/255, 111/255, 223/255),label='$qr1$')
plt.plot(qd1, c='gray',label='$qd1$')
plt.plot(q1, color=(241/255, 64/255, 64/255),label='$q1$')
plt.plot(hat_q1, c='green',label='$\hat q_1$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(qr2, color=(26/255, 111/255, 223/255),label='$qr2$')
plt.plot(qd2, c='gray',label='$qd2$')
plt.plot(q2, color=(241/255, 64/255, 64/255),label='$q2$')
plt.plot(hat_q2, c='green',label='$\hat q_2$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(qr3, color=(26/255, 111/255, 223/255),label='$qr3$')
plt.plot(qd3, c='gray',label='$qd3$')
plt.plot(q3, color=(241/255, 64/255, 64/255),label='$q3$')
plt.plot(hat_q3, c='green',label='$\hat q_3$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(qr4, color=(26/255, 111/255, 223/255),label='$qr4$')
plt.plot(qd4, c='gray',label='$qd4$')
plt.plot(q4, color=(241/255, 64/255, 64/255),label='$q4$')
plt.plot(hat_q4, c='green',label='$\hat q_4$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(qr5, color=(26/255, 111/255, 223/255),label='$qr5$')
plt.plot(qd5, c='gray',label='$qd5$')
plt.plot(q5, color=(241/255, 64/255, 64/255),label='$q5$')
plt.plot(hat_q5, c='green',label='$\hat q_5$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(qr6, color=(26/255, 111/255, 223/255),label='$qr6$')
plt.plot(qd6, c='gray',label='$qd6$')
plt.plot(q6, color=(241/255, 64/255, 64/255),label='$q6$')
plt.plot(hat_q6, c='green',label='$\hat q_6$')
plt.legend()
plt.show()

plt.subplot(4,2,1)
plt.plot(dq0, color=(26/255, 111/255, 223/255),label='$dq_0$')
plt.plot(hat_dq0, color=(241/255, 64/255, 64/255),label='$\hat dq_0$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(dq1, color=(26/255, 111/255, 223/255),label='$dq_1$')
plt.plot(hat_dq1, color=(241/255, 64/255, 64/255),label='$\hat dq_1$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(dq2, color=(26/255, 111/255, 223/255),label='$dq_2$')
plt.plot(hat_dq2, color=(241/255, 64/255, 64/255),label='$\hat dq_2$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(dq3, color=(26/255, 111/255, 223/255),label='$dq_3$')
plt.plot(hat_dq3, color=(241/255, 64/255, 64/255),label='$\hat dq_3$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(dq4, color=(26/255, 111/255, 223/255),label='$dq_4$')
plt.plot(hat_dq4, color=(241/255, 64/255, 64/255),label='$\hat dq_4$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(dq5, color=(26/255, 111/255, 223/255),label='$dq_5$')
plt.plot(hat_dq5, color=(241/255, 64/255, 64/255),label='$\hat dq_5$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(dq6, color=(26/255, 111/255, 223/255),label='$dq_6$')
plt.plot(hat_dq6, color=(241/255, 64/255, 64/255),label='$\hat dq_6$')
plt.legend()
plt.show()

plt.subplot(4,2,1)
plt.plot(f0, color=(26/255, 111/255, 223/255),label='$f_0$')
plt.plot(hat_f0, color=(241/255, 64/255, 64/255),label='$\hat f_0$')
plt.legend()
plt.subplot(4,2,2)
plt.plot(f1, color=(26/255, 111/255, 223/255),label='$f_1$')
plt.plot(hat_f1, color=(241/255, 64/255, 64/255),label='$\hat f_1$')
plt.legend()
plt.subplot(4,2,3)
plt.plot(f2, color=(26/255, 111/255, 223/255),label='$f_2$')
plt.plot(hat_f2, color=(241/255, 64/255, 64/255),label='$\hat f_2$')
plt.legend()
plt.subplot(4,2,4)
plt.plot(f3, color=(26/255, 111/255, 223/255),label='$f_3$')
plt.plot(hat_f3, color=(241/255, 64/255, 64/255),label='$\hat f_3$')
plt.legend()
plt.subplot(4,2,5)
plt.plot(f4, color=(26/255, 111/255, 223/255),label='$f_4$')
plt.plot(hat_f4, color=(241/255, 64/255, 64/255),label='$\hat f_4$')
plt.legend()
plt.subplot(4,2,6)
plt.plot(f5, color=(26/255, 111/255, 223/255),label='$f_5$')
plt.plot(hat_f5, color=(241/255, 64/255, 64/255),label='$\hat f_5$')
plt.legend()
plt.subplot(4,2,7)
plt.plot(f6, color=(26/255, 111/255, 223/255),label='$f_6$')
plt.plot(hat_f6, color=(241/255, 64/255, 64/255),label='$\hat f_6$')
plt.legend()
plt.show()


