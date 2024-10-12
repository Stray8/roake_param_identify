import numpy as np
import matplotlib.pyplot as plt


font1 = {'family' : 'Times new Roman',
'weight' : 'normal',
'size'   : 14,
}
plt.figure(figsize=(10, 8), dpi=100)


class Polish:
    def __init__(self, Period=0.01, mu_s=0.5, mu_c=0.4, c=0.2, f_push=10, D_trajectory = None):
        self.Period = Period
        self.mu_s = mu_s
        self.mu_c = mu_c
        self.c = c
        self.f_push = f_push
        self.D_trajectory = D_trajectory

        self.pv_array = []
        self.kx_array = []
        self.dx_array = []
        self.ky_array = []
        self.dy_array = []

        self.count = 0

    def step(self, pv, u, pv_r, f_c):
        '''
        :param pv, shape (4), px, vx, py, vy
        :param u: shape (6), -kx/mx, -dx/mx, 1/mx, -ky/my, -dy/my, 1/my
        :param pv_r, shape (4), dpx, dvx, dpy, dvy
        :param f_c, shape (2), f_cx, f_cy
        :return: next position, shape (4), px, vx, py, vy
        '''

        self.count = self.count + 1

        self.pv_array.append(pv)
        self.kx_array.append(np.array([-u[0] / u[2]]))
        self.dx_array.append(np.array([-u[1] / u[2]]))
        self.ky_array.append(np.array([-u[3] / u[5]]))
        self.dy_array.append(np.array([-u[4] / u[5]]))

        px, vx, py, vy = pv
        dpx, dvx, dpy, dvy = pv_r
        f_cx, f_cy = f_c
        epx = px - dpx
        evx = vx - dvx
        epy = py - dpy
        evy = vy - dvy

        fx_active = u[0] / u[2] * epx + u[1] / u[2] * evx  # u[0] / u[2] is -kx
        fy_active = u[3] / u[5] * epy + u[4] / u[5] * evy
        f_active_amplitude = np.sqrt(fx_active * fx_active + fy_active * fy_active)
        v_amplitude = np.sqrt(vx * vx + vy * vy)

        if v_amplitude == 0 and f_active_amplitude <= self.mu_s * self.f_push:
            f_amplitude = f_active_amplitude
            fx = -f_amplitude / f_active_amplitude * fx_active
            fy = -f_amplitude / f_active_amplitude * fy_active

        elif v_amplitude == 0 and f_active_amplitude > self.mu_s * self.f_push:
            f_amplitude = self.mu_s * self.f_push
            fx = -f_amplitude / f_active_amplitude * fx_active
            fy = -f_amplitude / f_active_amplitude * fy_active

        else:
            f_amplitude = self.mu_c * self.f_push + self.c * v_amplitude
            fx = -f_amplitude / v_amplitude * vx
            fy = -f_amplitude / v_amplitude * vy

        ax = (fx_active + fx + f_cx) * u[2]
        ay = (fy_active + fy + f_cy) * u[5]

        px = px + vx * self.Period + 0.5 * ax * self.Period * self.Period
        py = py + vy * self.Period + 0.5 * ay * self.Period * self.Period
        vx = vx + self.Period * ax
        vy = vy + self.Period * ay
        return np.array([px, vx, py, vy, fx, fy])

    def plot_trajectory(self, Stop_Flag=False, save_dir=None):
        plt.ion()
        plt.clf()
        position_array = np.array(self.pv_array)
        position_array_x = position_array[:, 0:2]
        position_array_y = position_array[:, 2:4]

        kx_array = np.array(self.kx_array)
        dx_array = np.array(self.dx_array)
        ky_array = np.array(self.ky_array)
        dy_array = np.array(self.dy_array)

        '''
        D_Trajectory_x = self.D_Trajectory[:, 0:2]
        D_Trajectory_y = self.D_Trajectory[:, 2:4]

        plt.subplot(2, 2, 1)
        plt.plot(D_Trajectory_x[0:self.count, 0],  D_Trajectory_y[0:self.count, 0], ls='--')
        plt.plot(position_array_x[:, 0], position_array_y[:, 0], ls='-')

        plt.subplot(2, 2, 2)
        plt.plot(np.arange(np.shape(kx_array)[0]), kx_array, c='red')
        plt.plot(np.arange(np.shape(ky_array)[0]), ky_array, c='blue')

        plt.subplot(2, 2, 3)
        plt.plot(np.arange(np.shape(dx_array)[0]), dx_array, c='red')
        plt.plot(np.arange(np.shape(dy_array)[0]), dy_array, c='blue')

        plt.subplot(2, 2, 4)
        plt.plot(np.arange(np.shape(mx_array)[0]), mx_array, c='red')
        plt.plot(np.arange(np.shape(my_array)[0]), my_array, c='blue')
        '''

        plt.subplots_adjust(left=0.1, right=0.99, wspace=0.35, hspace=0.52, bottom=0.1, top=0.99)
        plt1 = plt.subplot2grid((4, 4), (0, 0), rowspan=2, colspan=2)
        plt2 = plt.subplot2grid((4, 4), (0, 2), colspan=2)
        plt3 = plt.subplot2grid((4, 4), (1, 2), colspan=2)
        plt4 = plt.subplot2grid((4, 4), (2, 0), rowspan=1, colspan=4)
        plt5 = plt.subplot2grid((4, 4), (3, 0), rowspan=1, colspan=4)

        dpx = self.D_trajectory[:, 0]
        dpy = self.D_trajectory[:, 2]
        px = position_array_x[:, 0]
        py = position_array_y[:, 0]

        plt1.plot(dpx, dpy, ls='--', c='red', label='${\\xi}_r$')
        # plt1.plot(rx, ry, ls='--', c='red')
        plt1.plot(px, py, ls='-', c='blue', label='$\\xi$')
        thres_y = np.arange(0, 0.11, 0.01)
        thres_x = np.ones(np.shape(thres_y)[0]) * 0.41
        # plt1.plot(thres_x, thres_y, ls=':', c='grey')
        plt1.set_xlabel('$x$ (m)', fontname='Times New Roman', fontsize=12)
        plt1.set_ylabel('$y$ (m)', fontname='Times New Roman', fontsize=12)

        plt1.legend(loc='lower right', prop=font1, frameon=False, handlelength=4, ncol=2, columnspacing=1)
        plt1.set_title('(a): Motion trajectory', y=-0.21, fontname='Times New Roman', fontsize=12)
        plt1.tick_params(labelsize=12)

        plt2.plot(np.arange(np.shape(dpx)[0]), dpx, c='red', label='$x_r$')
        plt2.plot(np.arange(np.shape(px)[0]), px, c='blue', label='$x$')
        thres = np.ones(np.shape(px)[0]) * 0.41
        # plt2.plot(np.arange(np.shape(thres)[0]), thres, c='grey', ls=':')
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

        thres_h = np.ones(np.shape(px)[0])
        thres_l = np.ones(np.shape(px)[0]) * 100
        plt4.plot(np.arange(np.shape(dx_array)[0]), dx_array, c='red', label='$d_x$')
        plt4.plot(np.arange(np.shape(dy_array)[0]), dy_array, c='blue', label='$d_y$')
        # plt4.plot(np.arange(np.shape(thres_h)[0]), thres_h, c='grey', ls=':')
        # plt4.plot(np.arange(np.shape(thres_l)[0]), thres_l, c='grey', ls=':')
        plt4.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
        plt4.set_ylabel('Damping $d$ (Ns/m)', fontname='Times New Roman', fontsize=12)
        plt4.set_title('(d): Damping trajectory', y=-0.55, fontname='Times New Roman', fontsize=12)
        plt4.legend(loc='upper left', prop=font1, frameon=False, handlelength=1, ncol=1, columnspacing=1)
        plt4.tick_params(labelsize=12)

        thres_h = np.ones(np.shape(px)[0]) * 100
        thres_l = np.ones(np.shape(px)[0]) * 0
        plt5.plot(np.arange(np.shape(kx_array)[0]), kx_array, c='red', label='$k_x$')
        plt5.plot(np.arange(np.shape(ky_array)[0]), ky_array, c='blue', label='$k_y$')
        # plt5.plot(np.arange(np.shape(thres_h)[0]), thres_h, c='grey', ls=':')
        # plt5.plot(np.arange(np.shape(thres_l)[0]), thres_l, c='grey', ls=':')
        plt5.set_xlabel('steps', fontname='Times New Roman', fontsize=12)
        plt5.set_ylabel('Stiffness $k$ (N/m)', fontname='Times New Roman', fontsize=12)
        plt5.set_title('(e): Stiffness trajectory', y=-0.55, fontname='Times New Roman', fontsize=12)
        plt5.legend(loc='upper left', prop=font1, frameon=False, handlelength=1, ncol=1, columnspacing=1)
        plt5.tick_params(labelsize=12)

        plt.pause(0.001)

        if save_dir is None:
            pass
        else:
            plt.savefig(save_dir + str(self.count) + ".jpg", dpi=100)
        if Stop_Flag is True:
            print('simulation complete')
            plt.ioff()
            plt.show()
