import numpy as np


def rotation(coord, theta):
    c, s = np.cos(theta), np.sin(theta)

    rot_matrix = None

    if coord == 'z':
        rot_matrix = np.array(((s, -c, 0, 0), (c, s, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

    elif coord == 'y':
        rot_matrix = np.array(((c, 0, s, 0), (0, 1, 0, 0), (-s, 0, c, 0), (0, 0, 0, 1)))

    elif coord == 'x':
        rot_matrix = np.array(((1, 0, 0, 0), (0, c, -s, 0), (0, s, c, 0), (0, 0, 0, 1)))

    return rot_matrix


def transition(coord, d):

    trans_matrix = np.eye(4, dtype=float)

    if coord == 'z':
        trans_matrix[2][3] = d

    if coord == 'y':
        trans_matrix[1][3] = d

    if coord == 'x':
        trans_matrix[0][3] = d

    return trans_matrix


def coord_vector(d):
    return np.array(((0), (0), (d), (1)))


def projection():
    fov = 90
    z_near = -200.0
    z_far = 200.0
    ar = 1000.0 / 1000.0
    fov_rad = 1 / np.tan(fov * 0.5 / 180.0 * np.pi)

    proj_mat = np.zeros([4, 4])

    proj_mat[0][0] = ar * fov_rad
    proj_mat[1][1] = fov_rad
    proj_mat[2][2] = z_far / (z_far - z_near)
    proj_mat[3][2] = (-z_far * z_near) / (z_far - z_near)
    proj_mat[2][3] = 1.0
    proj_mat[3][3] = 0.0

    return proj_mat










