import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, Slider, CheckButtons
from pandas.plotting import register_matplotlib_converters
import matrix_library
import math
import matplotlib.style

register_matplotlib_converters()
matplotlib.style.use('ggplot')


class Robot_Arm_Gui:
    def __init__(self):
        self.fi_1_limit = [-np.pi / 2, np.pi / 2]
        self.fi_2_limit = [-11 / 36 * np.pi, 25 / 36 * np.pi]
        self.fi_3_limit = [0, 5 / 6 * np.pi]

        self.l1 = 203
        self.l2 = 178
        self.l3 = 178
        self.step = 0.1

        self.old_fi_1 = None
        self.old_fi_2 = None
        self.old_fi_3 = None

        self.xvalues_C_obalka_1 = []
        self.xvalues_C_obalka_2 = []

        self.yvalues_C_obalka_1 = []
        self.yvalues_C_obalka_2 = []

        self.zvalues_C_obalka_1 = []
        self.zvalues_C_obalka_2 = []

        self.A_coord = []
        self.B_coord = []
        self.C_coord = []

        self.radio_obalka = None
        self.button_axes = None
        self.slider_1 = None
        self.slider_2 = None
        self.slider_3 = None

        self.func_obalka = None
        self.plotted_axes = [False, False, False, False, False, False, False]
        self.subplot_axes = []

        self.obalka = True

        self.create_edge()

    # Function which reverse list
    @staticmethod
    def reverse(s):
        return s[::-1]

    # Update arm coordinates for new f1, f2, f3
    def update_arm(self, fi1, fi2, fi3):

        # If fi_1 is not passed to function, use old fi_1
        if fi1 is None:
            fi1 = self.old_fi_1

        # If fi_12 is not passed to function, use old fi_2
        if fi2 is None:
            fi2 = self.old_fi_2

        # If fi_3 is not passed to function, use old fi_3
        if fi3 is None:
            fi3 = self.old_fi_3

        # Rotation matrices
        r_z_fi1 = matrix_library.rotation('z', fi1)
        r_y_fi2 = matrix_library.rotation('y', fi2)
        r_y_fi3 = matrix_library.rotation('y', fi3)

        # Translation matrices
        t_z_l1 = matrix_library.transition('z', self.l1)
        t_z_l2 = matrix_library.transition('z', self.l2)

        # Compute new coordinates
        self.A_coord = np.matmul(r_z_fi1, matrix_library.coord_vector(self.l1))
        self.B_coord = np.matmul(np.matmul(r_z_fi1, t_z_l1), np.matmul(r_y_fi2, matrix_library.coord_vector(self.l2)))
        C_coord_temp = np.matmul(np.matmul(r_z_fi1, t_z_l1), np.matmul(r_y_fi2, t_z_l2))
        self.C_coord = np.matmul(C_coord_temp, np.matmul(r_y_fi3, matrix_library.coord_vector(self.l3)))

        # Saving old angles
        self.old_fi_1 = fi1
        self.old_fi_2 = fi2
        self.old_fi_3 = fi3

    # Function which plots robotic arm to 3D plane with specified angle to specified ax
    def plot_arm(self, ax, angle):

        # Plot obalka if radio button is selected for it
        if self.obalka:
            ax.plot(self.xvalues_C_obalka_1, self.yvalues_C_obalka_1, self.zvalues_C_obalka_1, color='blue')
            ax.plot(self.xvalues_C_obalka_1, self.yvalues_C_obalka_1, self.zvalues_C_obalka_1, color='blue')
            ax.plot(self.xvalues_C_obalka_2, self.yvalues_C_obalka_2, self.zvalues_C_obalka_2, color='green')

        # Plot joins
        ax.scatter3D([self.A_coord[0]], [self.A_coord[1]], [self.A_coord[2]], 'o', color='red', s=34)
        ax.scatter3D([self.B_coord[0]], [self.B_coord[1]], [self.B_coord[2]], 'o', color='red', s=34)
        ax.scatter3D([self.C_coord[0]], [self.C_coord[1]], [self.C_coord[2]], 'o', color='red', s=34)

        # Plot arms
        ax.plot([0, self.A_coord[0]], [0, self.A_coord[1]], [0, self.A_coord[2]], color='black', linewidth=4)
        ax.plot([self.A_coord[0], self.B_coord[0]], [self.A_coord[1], self.B_coord[1]], [self.A_coord[2], self.B_coord[2]], color='black', linewidth=4)
        ax.plot([self.B_coord[0], self.C_coord[0]], [self.B_coord[1], self.C_coord[1]], [self.B_coord[2], self.C_coord[2]], color='black', linewidth=4)

        # Plot coordinate axes
        if self.plotted_axes[0]:
            ax.plot([0, 0], [0, 0], [0, 80], color='green', linewidth=2)
            ax.plot([0, 0], [0, 80], [0, 0], color='yellow', linewidth=2)
            ax.plot([0, 80], [0, 0], [0, 0], color='blue', linewidth=2)

        if self.plotted_axes[1]:
            r = matrix_library.rotation('z', self.old_fi_1)
            a0_y = np.matmul(r, [0, 80, 0, 1])
            a0_x = np.matmul(r, [80, 0, 0, 1])
            a0_z = np.matmul(r, [0, 0, 80, 1])
            ax.plot([0, a0_z[0]], [0, a0_z[1]], [0, a0_z[2]], color='green', linewidth=2)
            ax.plot([0, a0_y[0]], [0, a0_y[1]], [0, a0_y[2]], color='orange', linewidth=2)
            ax.plot([0, a0_x[0]], [0, a0_x[1]], [0, a0_x[2]], color='blue', linewidth=2)

        if self.plotted_axes[2]:
            r1 = matrix_library.rotation('z', self.old_fi_1)
            a0_y = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), [0, 80, 0, 1])
            a0_x = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), [80, 0, 0, 1])
            a0_z = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), [0, 0, 80, 1])
            ax.plot([self.A_coord[0], a0_z[0]], [self.A_coord[1], a0_z[1]], [self.A_coord[2], a0_z[2]], color='green', linewidth=2)
            ax.plot([self.A_coord[0], a0_y[0]], [self.A_coord[1], a0_y[1]], [self.A_coord[2], a0_y[2]], color='orange',linewidth=2)
            ax.plot([self.A_coord[0], a0_x[0]], [self.A_coord[1], a0_x[1]], [self.A_coord[2], a0_x[2]], color='blue',linewidth=2)

        if self.plotted_axes[3]:
            r1 = matrix_library.rotation('z', self.old_fi_1)
            r2 = matrix_library.rotation('y', self.old_fi_2)
            a0_y = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(r2, [0, 80, 0, 1]))
            a0_x = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(r2, [80, 0, 0, 1]))
            a0_z = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(r2, [0, 0, 80, 1]))
            ax.plot([self.A_coord[0], a0_z[0]], [self.A_coord[1], a0_z[1]], [self.A_coord[2], a0_z[2]], color='green', linewidth=2)
            ax.plot([self.A_coord[0], a0_y[0]], [self.A_coord[1], a0_y[1]], [self.A_coord[2], a0_y[2]], color='orange', linewidth=2)
            ax.plot([self.A_coord[0], a0_x[0]], [self.A_coord[1], a0_x[1]], [self.A_coord[2], a0_x[2]], color='blue', linewidth=2)

        if self.plotted_axes[4]:
            r1 = matrix_library.rotation('z', self.old_fi_1)
            r2 = matrix_library.rotation('y', self.old_fi_2)
            a0_y = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), [0, 80, 0, 1]))
            a0_x = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), [80, 0, 0, 1]))
            a0_z = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), [0, 0, 80, 1]))
            ax.plot([self.B_coord[0], a0_z[0]], [self.B_coord[1], a0_z[1]], [self.B_coord[2], a0_z[2]], color='green', linewidth=2)
            ax.plot([self.B_coord[0], a0_y[0]], [self.B_coord[1], a0_y[1]], [self.B_coord[2], a0_y[2]], color='orange', linewidth=2)
            ax.plot([self.B_coord[0], a0_x[0]], [self.B_coord[1], a0_x[1]], [self.B_coord[2], a0_x[2]], color='blue', linewidth=2)

        if self.plotted_axes[5]:
            r1 = matrix_library.rotation('z', self.old_fi_1)
            r2 = matrix_library.rotation('y', self.old_fi_2)
            r3 = matrix_library.rotation('y', self.old_fi_3)
            a0_y = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(r3, [0, 80, 0, 1])))
            a0_x = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(r3, [80, 0, 0, 1])))
            a0_z = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(r3, [0, 0, 80, 1])))
            ax.plot([self.B_coord[0], a0_z[0]], [self.B_coord[1], a0_z[1]], [self.B_coord[2], a0_z[2]], color='green', linewidth=2)
            ax.plot([self.B_coord[0], a0_y[0]], [self.B_coord[1], a0_y[1]], [self.B_coord[2], a0_y[2]], color='orange', linewidth=2)
            ax.plot([self.B_coord[0], a0_x[0]], [self.B_coord[1], a0_x[1]], [self.B_coord[2], a0_x[2]], color='blue', linewidth=2)

        if self.plotted_axes[6]:
            r1 = matrix_library.rotation('z', self.old_fi_1)
            r2 = matrix_library.rotation('y', self.old_fi_2)
            r3 = matrix_library.rotation('y', self.old_fi_3)
            a0_y = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(np.matmul(r3, matrix_library.transition('z', self.l3)), [0, 80, 0, 1])))
            a0_x = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(np.matmul(r3, matrix_library.transition('z', self.l3)), [80, 0, 0, 1])))
            a0_z = np.matmul(np.matmul(r1, matrix_library.transition('z', self.l1)), np.matmul(np.matmul(r2, matrix_library.transition('z', self.l2)), np.matmul(np.matmul(r3, matrix_library.transition('z', self.l3)), [0, 0, 80, 1])))
            ax.plot([self.C_coord[0], a0_z[0]], [self.C_coord[1], a0_z[1]], [self.C_coord[2], a0_z[2]], color='green', linewidth=2)
            ax.plot([self.C_coord[0], a0_y[0]], [self.C_coord[1], a0_y[1]], [self.C_coord[2], a0_y[2]], color='orange', linewidth=2)
            ax.plot([self.C_coord[0], a0_x[0]], [self.C_coord[1], a0_x[1]], [self.C_coord[2], a0_x[2]], color='blue', linewidth=2)

        # Plot coordinates
        ax.set_xlabel('x', fontweight='bold')
        ax.set_ylabel('y', fontweight='bold')
        ax.set_zlabel('z', fontweight='bold')
        ax.set_xlim(-400, 400)
        ax.set_ylim(-400, 400)
        ax.set_zlim(-100, 400)
        ax.set_yticks([-300, 0, 300])
        ax.set_xticks([-300, 0, 300])
        ax.set_zticks([0, 300])
        ax.view_init(30, angle)
        plt.draw()

    def setup_gui(self):

        # Setting figure
        fig1 = plt.figure(figsize=(10,9)) 
        fig1.subplots_adjust(left=0.3, wspace=0.2, hspace=0.3)

        # Setting up axes
        ax = fig1.add_subplot(2, 2, 1, projection="3d")
        bx = fig1.add_subplot(2, 2, 2, projection="3d")
        cx = fig1.add_subplot(2, 2, 3, projection="3d")
        dx = fig1.add_subplot(2, 2, 4, projection="3d")

        # Append axes to list of axes
        self.subplot_axes.append(ax)
        self.subplot_axes.append(bx)
        self.subplot_axes.append(cx)
        self.subplot_axes.append(dx)

        # Title of figure
        fig1.suptitle('Zobrazenie robota v 3D priestore')
        fig1.text(0.11, 0.9, 'Volba sur. \nsystemov', rotation='horizontal', verticalalignment='center', fontsize='large')

        # Creating radio button for changing which series to plot
        rdb_obalka = plt.axes([0.05, 0.65, 0.15, 0.15])
        self.radio_obalka = RadioButtons(rdb_obalka, ('S obalkou', 'Bez obalky'))
        self.func_obalka = {'S obalkou': True, 'Bez obalky': False}

        # Creating check box for axes
        btn_axes = plt.axes([0.05, 0.83, 0.15, 0.15])
        self.button_axes = CheckButtons(btn_axes, ('0', '1', '2', '3', '4', '5', '6'))

        # Creating slider for changing filter constant
        fi_1_slider = plt.axes([0.05, 0.1, 0.03, 0.5])
        fi_2_slider = plt.axes([0.10, 0.1, 0.03, 0.5])
        fi_3_slider = plt.axes([0.15, 0.1, 0.03, 0.5])
        self.slider_1 = Slider(fi_1_slider, 'φ1 [°]', orientation='vertical', valmin=math.degrees(self.fi_1_limit[0]),valmax=math.degrees(self.fi_1_limit[1]),valinit=0.0, valstep=math.degrees(self.step))
        self.slider_2 = Slider(fi_2_slider, 'φ2 [°]', orientation='vertical', valmin=math.degrees(self.fi_2_limit[0]),valmax=math.degrees(self.fi_2_limit[1]),valinit=0.0, valstep=math.degrees(self.step))
        self.slider_3 = Slider(fi_3_slider, 'φ3 [°]', orientation='vertical', valmin=math.degrees(self.fi_3_limit[0]),valmax=math.degrees(self.fi_3_limit[1]),valinit=0.0, valstep=math.degrees(self.step))

        # Assign a function handler to a buttons and sliders
        self.radio_obalka.on_clicked(self.__radio_button_update)
        self.button_axes.on_clicked(self.__button_update)
        self.slider_1.on_changed(self.__slider_1_update)
        self.slider_2.on_changed(self.__slider_2_update)
        self.slider_3.on_changed(self.__slider_3_update)
        plt.draw()

        self.redraw_axes()

    # Handler for radio button
    def __radio_button_update(self, label):
        # Updating which data will be plotted
        self.obalka = self.func_obalka[label]
        self.redraw_axes()

    def __button_update(self, label):
        index = int(label)
        if self.plotted_axes[index]:
            self.plotted_axes[index] = False
        else:
            self.plotted_axes[index] = True
        self.redraw_axes()
        
    # Handler for slider 1
    def __slider_1_update(self, val):
        # Updating filter constant
        fi_1 = math.radians(self.slider_1.val)
        # Updating subplots by class function
        self.update_arm(fi_1, None, None)
        self.redraw_axes()

    # Handler for slider 2
    def __slider_2_update(self, val):
        # Updating filter constant
        fi_2 = math.radians(self.slider_2.val)
        # Updating subplots by class function
        self.update_arm(None, fi_2, None)
        self.redraw_axes()

    # Handler for slider 3
    def __slider_3_update(self, val):
        # Updating filter constant
        fi_3 = math.radians(self.slider_3.val)
        # Updating subplots by class function
        self.update_arm(None, None, fi_3)
        self.redraw_axes()

    def redraw_axes(self):
        for a, i in zip(self.subplot_axes, range(0, 181, 60)):
            a.clear()
            self.plot_arm(a, i)
            a.set_xlabel('x')
            a.set_ylabel('y')
            a.set_zlabel('z')

    def create_edge(self):
        data_C_obalka_1 = []
        data_C_obalka_2 = []
        temp_list = []

        for k in np.linspace(self.fi_3_limit[0], self.fi_3_limit[1], int((self.fi_3_limit[1] - self.fi_3_limit[0]) / self.step) + 1):
            for j in np.linspace(self.fi_2_limit[0], self.fi_2_limit[1], int((self.fi_2_limit[1] - self.fi_2_limit[0]) / self.step) + 1):
                for i in np.linspace(self.fi_1_limit[0], self.fi_1_limit[1], int((self.fi_1_limit[1] - self.fi_1_limit[0]) / self.step) + 1):

                    r_z_fi1 = matrix_library.rotation('z', i)
                    r_y_fi2 = matrix_library.rotation('y', j)
                    r_y_fi3 = matrix_library.rotation('y', k)

                    t_z_l1 = matrix_library.transition('z', self.l1)
                    t_z_l2 = matrix_library.transition('z', self.l2)

                    C_coord_temp = np.matmul(np.matmul(r_z_fi1, t_z_l1), np.matmul(r_y_fi2, t_z_l2))
                    C_coord = np.matmul(C_coord_temp, np.matmul(r_y_fi3, matrix_library.coord_vector(self.l3)))

                    # append only if point is on the edge

                    # horizontal edge map for fi_2 > 0
                    if (abs(j - (np.pi / 2)) < self.step) or (abs(j - self.fi_2_limit[0]) < self.step):
                        if abs(k - 0) < self.step:
                            data_C_obalka_1.append(C_coord)

                    # horizontal edge map for fi_2 > 0
                    if abs(i - self.fi_1_limit[0]) < self.step or abs(i - self.fi_1_limit[1]) < self.step:
                        data_C_obalka_1.append(C_coord)

                    # vertical edge map for every fi_3 == 0
                    if abs(k - 0) < self.step:
                        if abs(i - 0) < self.step:
                            data_C_obalka_2.append(C_coord)

                    # vertical edge map for every fi_3 when fi_2 == max
                    if abs(j - self.fi_2_limit[1]) < self.step:
                        if abs(i - 0) < self.step:
                            data_C_obalka_2.append(C_coord)

                    # vertical edge map for every fi_3 when fi_2 == min
                    if abs(j - self.fi_2_limit[0]) < self.step:
                        if abs(i - 0) < self.step:
                            temp_list.append(C_coord)

        temp_list = self.reverse(temp_list)
        data_C_obalka_2.extend(temp_list)

        for c in data_C_obalka_1:
            self.xvalues_C_obalka_1.append(c[0]), self.yvalues_C_obalka_1.append(c[1]), self.zvalues_C_obalka_1.append(0)

        for c in data_C_obalka_2:
            self.xvalues_C_obalka_2.append(0), self.yvalues_C_obalka_2.append(c[1]), self.zvalues_C_obalka_2.append(c[2])


pass
