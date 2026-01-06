import numpy as np
import math
from modern_robotics import *


def genTrans_te(theta, distance, steps, start=-math.pi / 2, end=math.pi / 2):
    x_offset = distance * np.cos(theta)
    z_offset = distance * np.sin(theta)

    # from tool(object, target) frame to end-effector-0 (0 assume the word orientation)
    trans_te0 = np.array([[1, 0, 0, x_offset], [0, 1, 0, 0], [0, 0, 1, z_offset], [0, 0, 0, 1]])
    R_e0e = MatrixExp3(VecToso3(np.array([0, math.pi-theta, 0])))
    trans_e0e = RpToTrans(R_e0e, np.array([0, 0, 0]))

    omiga = np.linspace(start, end, steps)
    omiga = np.column_stack((np.zeros_like(omiga), np.zeros_like(omiga), omiga))

    R = []
    for o in omiga:
        R.append(MatrixExp3(VecToso3(o)))

    out = []
    for r in R:
        t = RpToTrans(r, np.array([0, 0, 0]))
        out.append(t @ trans_te0 @ trans_e0e)
    return out

def genTrans_se(theta,distance,steps,x_offset,start=-math.pi / 2, end=math.pi / 2):
    trans_te = genTrans_te(theta, distance, steps, start, end)
    T_st = RpToTrans(np.eye(3), np.array([x_offset, 0, 0]))
    T_se = T_st@trans_te
    return T_se



if __name__ == '__main__':
    out = genTrans_te(theta=np.deg2rad(45), distance=10, steps=2000,start=0)
    print(out[0])
    print(out[500])
    out = genTrans_se(theta=np.deg2rad(45), distance=0.2, steps=2000,start=0,x_offset=0.5)
    print(out[0])
    print(out[500])
