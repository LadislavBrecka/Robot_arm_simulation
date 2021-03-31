import pygame
import matrix_library
import numpy as np
from Colors import *


def plot_arrow(screen, font, x1, y1, x2, y2, label1, label2, swap_vertical=False, swap_horizontal=False):
    # arrow vertical
    if not swap_vertical:
        pygame.draw.line(screen, black, (x1, y1), (x1-10, y1+15))
        pygame.draw.line(screen, black, (x1, y1), (x1+10, y1+15))
    else:
        pygame.draw.line(screen, black, (x1, y1), (x1-10, y1-15))
        pygame.draw.line(screen, black, (x1, y1), (x1+10, y1-15))

    # arrow horizontal
    if not swap_horizontal:
        pygame.draw.line(screen, black, (x2, y2), (x2-15, y2-10))
        pygame.draw.line(screen, black, (x2, y2), (x2-15, y2+10))
    else:
        pygame.draw.line(screen, black, (x2, y2), (x2+15, y2-10))
        pygame.draw.line(screen, black, (x2, y2), (x2+15, y2+10))

    # labels vertical
    text1 = font.render(label1, False, (0, 0, 0))
    text2 = font.render(label2, False, (0, 0, 0))
    if not swap_vertical:
        screen.blit(text2, (x1 - 35, y1))
    else:
        screen.blit(text2, (x1 - 35, y1 - 35))

    if not swap_horizontal:
        screen.blit(text1, (x2 - 35, y2))
    else:
        screen.blit(text1, (x2 + 35, y2))


class RobotArm:

    def __init__(self):

        self.fi_1_limit = [-np.pi/2, np.pi/2]
        self.fi_2_limit = [-11/36*np.pi, 25/36*np.pi]
        self.fi_3_limit = [0, 5/6*np.pi]
        self.fi_1 = 0
        self.fi_2 = 0.0
        self.fi_3 = 0.0
        self.l1 = 203/3
        self.l2 = 178/3
        self.l3 = 178/3
        self.step = 0.2

        self.r_z_fi1 = matrix_library.rotation('z', 0)
        self.r_y_fi2 = matrix_library.rotation('y', 0)
        self.r_y_fi3 = matrix_library.rotation('y', 0)

        self.t_z_l1 = matrix_library.transition('z', self.l1)
        self.t_z_l2 = matrix_library.transition('z', self.l2)

        self.A_coord = np.matmul(self.r_z_fi1, matrix_library.coord_vector(self.l1))
        self.B_coord = np.matmul(np.matmul(self.r_z_fi1, self.t_z_l1),
                                 np.matmul(self.r_y_fi2, matrix_library.coord_vector(self.l2)))
        C_coord_temp = np.matmul(np.matmul(self.r_z_fi1, self.t_z_l1), np.matmul(self.r_y_fi2, self.t_z_l2))
        self.C_coord = np.matmul(C_coord_temp, np.matmul(self.r_y_fi3, matrix_library.coord_vector(self.l3)))

        self.trajectory_graph1 = []
        self.trajectory_graph2 = []
        self.points_graph_1 = []
        self.points_graph_2 = []
        self.loop = True
        self.old_fi_1 = self.fi_1
        self.proj_mat = matrix_library.projection()

        self.width = 1000.0
        self.height = 1000.0
        self.distance = 5.0
        self.scale = 200
        self.position = [750, 250]

        # Draw related stuff
        self.my_font1 = pygame.font.SysFont("monospace", 30)

    def update(self):

        old_A = self.A_coord
        old_B = self.B_coord
        old_C = self.C_coord
        self.old_fi_1 = self.fi_1

        self.r_z_fi1 = matrix_library.rotation('z', self.fi_1)
        self.r_y_fi2 = matrix_library.rotation('y', self.fi_2)
        self.r_y_fi3 = matrix_library.rotation('y', self.fi_3)

        self.t_z_l1 = matrix_library.transition('z', self.l1)
        self.t_z_l2 = matrix_library.transition('z', self.l2)

        self.A_coord = np.matmul(self.r_z_fi1, matrix_library.coord_vector(self.l1))
        self.B_coord = np.matmul(np.matmul(self.r_z_fi1, self.t_z_l1), np.matmul(self.r_y_fi2, matrix_library.coord_vector(self.l2)))
        C_coord_temp = np.matmul(np.matmul(self.r_z_fi1, self.t_z_l1), np.matmul(self.r_y_fi2, self.t_z_l2))
        self.C_coord = np.matmul(C_coord_temp, np.matmul(self.r_y_fi3, matrix_library.coord_vector(self.l3)))

        # ordered iterating
        if self.loop:
            self.fi_1 = self.fi_1 + self.step
            if self.fi_1 > self.fi_1_limit[1]:
                self.fi_1 = self.fi_1_limit[0]
                self.fi_2 = self.fi_2 + self.step
                if self.fi_2 > self.fi_2_limit[1]:
                    self.fi_2 = self.fi_2_limit[0]
                    self.fi_3 = self.fi_3 + self.step
                    if self.fi_3 > self.fi_3_limit[1]:
                        self.loop = False

        return old_A, old_B, old_C

    def draw(self, screen):

        old_A, old_B, old_C = self.update()

        offset_x_graph_2 = 750 - 20
        offset_y_graph_2 = 250 - 20

        offset_x_graph_1 = 250 + 20
        offset_y_graph_1 = 250 - 20

        for p in self.points_graph_1:
            pygame.draw.circle(screen, green, p, 3)

        for p in self.points_graph_2:
            pygame.draw.circle(screen, green, p, 3)

        '''
        y-z graph
        '''
        # axes x
        pygame.draw.line(screen, black, (offset_x_graph_1, offset_y_graph_1),(offset_x_graph_1 + 250, offset_y_graph_1))
        # axes y
        pygame.draw.line(screen, black, (offset_x_graph_1, 0), (offset_x_graph_1, offset_y_graph_2))
        # arrows
        plot_arrow(screen, self.my_font1, offset_x_graph_1, 0, offset_x_graph_1+250, offset_y_graph_1, label1='y', label2='z')

        # lines between points
        pygame.draw.line(screen, black, (offset_x_graph_1, offset_y_graph_1), ((self.A_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.A_coord[2])), width=5)
        pygame.draw.line(screen, black, ((self.A_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.A_coord[2])), ((self.B_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.B_coord[2])), width=5)
        pygame.draw.line(screen, black, ((self.B_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.B_coord[2])), ((self.C_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.C_coord[2])), width=5)

        # points
        pygame.draw.circle(screen, red, ((self.A_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.A_coord[2])), 5)
        pygame.draw.circle(screen, red, ((self.B_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.B_coord[2])), 5)
        pygame.draw.circle(screen, red, ((self.C_coord[1] + offset_x_graph_1), (offset_y_graph_1 - self.C_coord[2])), 5)
        self.points_graph_1.append([self.C_coord[1] + offset_x_graph_1, offset_y_graph_1 - self.C_coord[2]])

        '''
        y-x graph
        '''
        # axes y
        pygame.draw.line(screen, black, (offset_x_graph_2, offset_y_graph_2), (offset_x_graph_2+250, offset_y_graph_2))
        # axes y
        pygame.draw.line(screen, black, (offset_x_graph_2, offset_y_graph_2), (offset_x_graph_2, offset_y_graph_2+250))
        # arrows
        plot_arrow(screen, self.my_font1, offset_x_graph_2, offset_y_graph_2+250, offset_x_graph_2 + 250, offset_y_graph_2, label1='y', label2='x', swap_vertical=True)

        # lines between points
        pygame.draw.line(screen, black, (offset_x_graph_2, offset_y_graph_2), (self.A_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.A_coord[0]), width=5)
        pygame.draw.line(screen, black, (self.A_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.A_coord[0]), (self.B_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.B_coord[0]), width=5)
        pygame.draw.line(screen, black, (self.B_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.B_coord[0]), (self.C_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.C_coord[0]), width=5)

        # points
        pygame.draw.circle(screen, red, (self.A_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.A_coord[0]), 5)
        pygame.draw.circle(screen, red, (self.B_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.B_coord[0]), 5)
        pygame.draw.circle(screen, red, (self.C_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.C_coord[0]), 5)
        self.points_graph_2.append([self.C_coord[1] + offset_x_graph_2, offset_y_graph_2 - self.C_coord[0]])


