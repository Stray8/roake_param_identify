import numpy as np
import matplotlib.pyplot as plt

font1 = {'family' : 'Times new Roman',
'weight' : 'normal',
'size'   : 14,
}

collected_dataSet = np.loadtxt('collected_dataSet.txt')
D_trajectory = np.loadtxt('D_trajectory.txt')[0:600, :]
dpx = D_trajectory[:, 0]
dpy = D_trajectory[:, 2]

px = collected_dataSet[0:600, 0]
vx = collected_dataSet[0:600, 1]
fx = collected_dataSet[0:600, 2]
fc_x = collected_dataSet[0:600, 3]
py = collected_dataSet[0:600, 4]
vy = collected_dataSet[0:600, 5]
fy = collected_dataSet[0:600, 6]
fc_y = collected_dataSet[0:600, 7]
pz = collected_dataSet[0:600, 8]
vz = collected_dataSet[0:600, 9]
fz = collected_dataSet[0:600, 10]
mx = 1 / collected_dataSet[0:600, 13]
dx = -collected_dataSet[0:600, 12] * mx
kx = -collected_dataSet[0:600, 11] * mx
my = 1 / collected_dataSet[0:600, 16]
dy = -collected_dataSet[0:600, 15] * my
ky = -collected_dataSet[0:600, 14] * my
mz = 1 / collected_dataSet[0:600, 19]
dz = -collected_dataSet[0:600, 18] * mz
kz = -collected_dataSet[0:600, 17] * mz
rx = collected_dataSet[0:600, 20]
ry = collected_dataSet[0:600, 21]
rz = collected_dataSet[0:600, 22]

plt.figure(figsize=(10, 8), dpi=100)
plt.subplots_adjust(left=0.1, right=0.99, wspace=0.35, hspace=0.52, bottom=0.1, top=0.99)
plt1 = plt.subplot2grid((4, 4), (0, 0), rowspan=2, colspan=2)
plt2 = plt.subplot2grid((4, 4), (0, 2), colspan=2)
plt3 = plt.subplot2grid((4, 4), (1, 2), colspan=2)
plt4 = plt.subplot2grid((4, 4), (2, 0), rowspan=1, colspan=4)
plt5 = plt.subplot2grid((4, 4), (3, 0), rowspan=1, colspan=4)

plt1.plot(dpx, dpy, ls='--', c='red', label='${\\xi}_r$')
# plt1.plot(rx, ry, ls='--', c='blue', label='${\\xi}_d$')
plt1.plot(px, py, ls='-', c='blue', label='$\\xi$')
thres_y = np.arange(-0.1, 0.1, 0.01)
thres_x = np.ones(np.shape(thres_y)[0]) * 0.42
plt1.plot(thres_x, thres_y, ls=':', c='grey')
plt1.set_xlabel('$x$ (m)', fontname='Times New Roman', fontsize=12)
plt1.set_ylabel('$y$ (m)', fontname='Times New Roman', fontsize=12)

plt1.legend(loc='lower right', prop=font1, frameon=False, handlelength=4, ncol=2, columnspacing=1)
plt1.set_title('(a): Motion trajectory', y=-0.21, fontname='Times New Roman', fontsize=12)
plt1.tick_params(labelsize=12)

plt2.plot(np.arange(np.shape(dpx)[0]), dpx, c='red', label='$x_r$')
plt2.plot(np.arange(np.shape(px)[0]), px, c='blue', label='$x$')
thres = np.ones(np.shape(rx)[0]) * 0.42
plt2.plot(np.arange(np.shape(thres)[0]), thres, c='grey', ls=':')
plt2.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
plt2.set_ylabel('$x$ (m)', fontname='Times New Roman', fontsize=12)
plt2.set_title('(b): $x$ trajectory', y=-0.55, fontname='Times New Roman', fontsize=12)
plt2.legend(loc='lower left', prop=font1, frameon=False, handlelength=4, ncol=2, columnspacing=1)
plt2.tick_params(labelsize=12)


plt3.plot(np.arange(np.shape(dpy)[0]), dpy, c='red', label='$y_r$')
plt3.plot(np.arange(np.shape(py)[0]), py, c='blue', label='$y$')
plt3.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
plt3.set_ylabel('$y$ (m)', fontname='Times New Roman', fontsize=12)
plt3.set_title('(c): $y$ trajectory', y=-0.55, fontname='Times New Roman', fontsize=12)
plt3.legend(loc='lower left', prop=font1, frameon=False, handlelength=4, ncol=2, columnspacing=1)
plt3.tick_params(labelsize=12)

thres_l = np.ones(np.shape(rx)[0]) * 2 * np.sqrt(200)
# plt4.plot(np.arange(np.shape(thres_l)[0]), thres_l, c='grey', ls=':')
thres_h = np.ones(np.shape(rx)[0]) * 2 * 1500 / np.sqrt(200)
# plt4.plot(np.arange(np.shape(thres_h)[0]), thres_h, c='grey', ls=':')
plt4.plot(np.arange(np.shape(fc_x)[0]), fc_x, c='red', ls='--', label='$f_{c,x}$')
plt4.plot(np.arange(np.shape(fc_y)[0]), fc_y, c='blue', ls='--', label='$f_{c,y}$')
plt4.plot(np.arange(np.shape(fx)[0]), fx, c='red', ls='-', label='$f_{x}$')
plt4.plot(np.arange(np.shape(fy)[0]), fy, c='blue', ls='-', label='$f_{y}$')
plt4.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
plt4.set_ylabel('Complementary forces (N)', fontname='Times New Roman', fontsize=12)
plt4.set_title('(d): Complementary force trajectories', y=-0.55, fontname='Times New Roman', fontsize=12)
plt4.legend(loc='upper left', prop=font1, frameon=False, handlelength=1, ncol=1, columnspacing=1)
plt4.tick_params(labelsize=12)

thres_l = np.ones(np.shape(rx)[0]) * 1000
plt5.plot(np.arange(np.shape(thres_l)[0]), thres_l, c='black', ls=':')
thres_h = np.ones(np.shape(rx)[0]) * 1500
plt5.plot(np.arange(np.shape(thres_h)[0]), thres_h, c='black', ls=':')
plt5.plot(np.arange(np.shape(kx)[0]), kx, c='red', label='$k_x$')
plt5.plot(np.arange(np.shape(ky)[0]), ky, c='blue', label='$k_y$')
plt5.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
plt5.set_ylabel('Stiffness $k$ (N/m)', fontname='Times New Roman', fontsize=12)
plt5.set_title('(e): Stiffness trajectory', y=-0.55, fontname='Times New Roman', fontsize=12)
plt5.legend(loc='upper left', prop=font1, frameon=False, handlelength=1, ncol=1, columnspacing=1)
plt5.tick_params(labelsize=12)

plt.savefig("E:/papers/拟投ICRA2022/simulation/figs/RectIPC42.png", dpi=150)
plt.savefig("E:/papers/拟投ICRA2022/simulation/figs/RectIPC42.pdf", dpi=1200)
plt.show()