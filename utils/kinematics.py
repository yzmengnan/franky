import numpy as np
from modern_robotics import *


def get_t_te(theta: float = 0.0, distance: float = 1, number: int = 2000, flag='deg'):
    """
    :param theta:
    :param distance:
    :param number:
    :param flag:
    :return: get transformation matrix from target to end-effector with 'number' steps
    """
    assert flag in ('rad', 'deg')
    if flag != 'rad':
        theta = np.deg2rad(theta)
    # first, rotate around xs with theta, get the posture
    rot_x = MatrixExp3(VecToso3(np.array([1, 0, 0]) * theta))
    # next, change the posture as the z-axis aimed to the object
    r2e = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]])
    # the rotation matrix
    R = rot_x @ r2e
    # the trans origin position
    x_pose = [0, distance * np.cos(theta), distance * np.sin(theta)]
    # M_0
    m = RpToTrans(R, x_pose)

    res = []
    z_twist = np.array([0, 0, 1, 0, 0, 0]).reshape(-1, 1)
    for t in np.linspace(-np.pi/2, np.pi/2, number).reshape(-1, 1):
        res.append(FKinSpace(m, z_twist, t))
    return res


def get_t_se(t_t_e, distance: float = 1,height=0.2 ):
    t = np.array([[1, 0, 0, 0], [0, 1, 0, distance], [0, 0, 1, height], [0, 0, 0, 1]])
    return t @ t_t_e
