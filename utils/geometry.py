from modern_robotics import *
from numpy import pi
import numpy as np


def transformation_t_e(distance, theta, steps, start=-pi / 2, end=pi / 2):
    # e_pos start from x_axis
    e_pos = np.array([distance * np.cos(theta), 0, distance * np.sin(theta)])
    # R_te= rotate around y-axis, +pi/2 means z-axis toward the target
    rotation_te = MatrixExp3(VecToso3(np.array([0, 1, 0]) * (theta + pi / 2)))
    # get m_e0
    m_e0 = RpToTrans(rotation_te, e_pos)
    # twist
    z_twist = np.array([0, 0, 1, 0, 0, 0]).reshape(-1, 1)
    trans_t_e = []
    for t in np.linspace(start, end, steps).reshape(-1, 1):
        out = FKinSpace(m_e0, z_twist, t)
        trans_t_e.append(out)
    return trans_t_e


def transformation_s_e(t_te, t_st):
    return t_st @ t_te


if __name__ == "__main__":
    t_te = transformation_t_e(1, np.deg2rad(45), 2000)
    print(t_te[1000])
