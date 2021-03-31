import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import RadioButtons, Slider
from pandas.plotting import register_matplotlib_converters
import matrix_library
register_matplotlib_converters()


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

        self.radio = None
        self.slider_1 = None
        self.slider_2 = None
        self.slider_3 = None
        self.func = None
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

        # Plot joins
        ax.scatter3D([self.A_coord[0]], [self.A_coord[1]], [self.A_coord[2]], 'o', color='red', s=34)
        ax.scatter3D([self.B_coord[0]], [self.B_coord[1]], [self.B_coord[2]], 'o', color='red', s=34)
        ax.scatter3D([self.C_coord[0]], [self.C_coord[1]], [self.C_coord[2]], 'o', color='red', s=34)

        # Plot arms
        ax.plot([0, self.A_coord[0]], [0, self.A_coord[1]], [0, self.A_coord[2]], color='black', linewidth=4)
        ax.plot([self.A_coord[0], self.B_coord[0]], [self.A_coord[1], self.B_coord[1]], [self.A_coord[2], self.B_coord[2]], color='black', linewidth=4)
        ax.plot([self.B_coord[0], self.C_coord[0]], [self.B_coord[1], self.C_coord[1]], [self.B_coord[2], self.C_coord[2]], color='black', linewidth=4)

        # Plot obalka if radio button is selected for it
        if self.obalka:
            ax.plot(self.xvalues_C_obalka_1, self.yvalues_C_obalka_1, self.zvalues_C_obalka_1, color='blue')
            ax.plot(self.xvalues_C_obalka_1, self.yvalues_C_obalka_1, self.zvalues_C_obalka_1, color='blue')
            ax.plot(self.xvalues_C_obalka_2, self.yvalues_C_obalka_2, self.zvalues_C_obalka_2, color='green')

        # Plot coordinates
        ax.set_xlabel('x')
        ax.set_ylabel('y')
        ax.set_zlabel('z')
        ax.set_xlim(-400,400)
        ax.set_ylim(-400,400)
        ax.set_zlim(-100,400)
        ax.view_init(30, angle)

        plt.draw()

    def setup_gui(self):

        # Setting figure
        fig1 = plt.figure(figsize=(10, 9))
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

        # Creating radio button for changing which series to plot
        rax = plt.axes([0.05, 0.7, 0.15, 0.15])
        self.radio = RadioButtons(rax, ('S obalkou', 'Bez obalky'))
        self.func = {'S obalkou': True, 'Bez obalky': False}

        # Creating slider for changing filter constant
        fi_1_slider = plt.axes([0.05, 0.1, 0.03, 0.5])
        fi_2_slider = plt.axes([0.10, 0.1, 0.03, 0.5])
        fi_3_slider = plt.axes([0.15, 0.1, 0.03, 0.5])
        self.slider_1 = Slider(fi_1_slider, 'Fi_1', orientation='vertical', valmin=self.fi_1_limit[0],valmax=self.fi_1_limit[1],valinit=0.0,valstep=self.step)
        self.slider_2 = Slider(fi_2_slider, 'Fi_2', orientation='vertical', valmin=self.fi_2_limit[0],valmax=self.fi_2_limit[1],valinit=0.0,valstep=self.step)
        self.slider_3 = Slider(fi_3_slider, 'Fi_3', orientation='vertical', valmin=self.fi_3_limit[0],valmax=self.fi_3_limit[1],valinit=0.0,valstep=self.step)

        # Assign a function handler to a button and slider
        self.radio.on_clicked(self.__radioButton_update)
        self.slider_1.on_changed(self.__slider_1_update)
        self.slider_2.on_changed(self.__slider_2_update)
        self.slider_3.on_changed(self.__slider_3_update)
        plt.draw()

        self.redraw_axes()

    # Handler for radio button
    def __radioButton_update(self, label):
        # Updating which data will be plotted
        self.obalka = self.func[label]
        print(self.obalka)
        self.redraw_axes()

    # Handler for slider 1
    def __slider_1_update(self, val):
        # Updating filter constant
        fi_1 = self.slider_1.val
        # Updating subplots by class function
        self.update_arm(fi_1, None, None)
        self.redraw_axes()

    # Handler for slider 2
    def __slider_2_update(self, val):
        # Updating filter constant
        fi_2 = self.slider_2.val
        # Updating subplots by class function
        self.update_arm(None, fi_2, None)
        self.redraw_axes()

    # Handler for slider 3
    def __slider_3_update(self, val):
        # Updating filter constant
        fi_3 = self.slider_3.val
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
