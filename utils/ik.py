import numpy as np
import pinocchio

from numpy.linalg import norm, solve
from trac_ik import TracIK
import logging
logger = logging.getLogger()


def ik_trac(model_path,base:str,target:str,seed_jnts,pos,rot):
    ik_solver = TracIK(base_link_name=base,tip_link_name=target,urdf_path=model_path)
    result = ik_solver.ik(tgt_pos=pos, tgt_rot=rot, seed_jnt_values=seed_jnts)
    if result is not None:
        return result
    else:
        logger.warning("No result from ik_solver")
        return None

class IK_Traj:
    def __init__(self,urdf_path,target_frame,base_frame):
        self.urdf_path = urdf_path
        self.ik_solver = TracIK(base_link_name=base_frame,tip_link_name=target_frame,urdf_path=urdf_path)

    def __get_ik_results(self,seed_jnt,pos,rot):
        res = self.ik_solver.ik(tgt_pos=pos, tgt_rot=rot, seed_jnt_values=seed_jnt)
        return res
    def gen_trajectory(self,start_qpos,t_m):
        seed = np.zeros_like(start_qpos)
        traj = []
        for i,elem in enumerate(t_m):
            pos = elem[:3,-1]
            rot = elem[:3,:3]
            if i==0:
                res = self.__get_ik_results(seed_jnt=start_qpos,pos=pos,rot=rot)
            else:
                res = self.__get_ik_results(seed_jnt=seed,pos=pos,rot=rot)
            if res is not None:
                assert "error in solving inverse kinematics"
            traj.append(res)
            seed = res
        return traj

    def gen_qpos_realtime(self,start_qpos,t_m):
        pos = t_m[:3,-1]
        rot = t_m[:3,:3]
        res = self.__get_ik_results(seed_jnt=start_qpos,pos=pos,rot=rot)
        return res


def IK(model, data, T, eps=1e-4, IT_MAX=1000, DT=1e-1, damp=1e-12):
    """
    :description: based on pinocchio examples
    :param model:
    :param data:
    :param T:
    :param eps:
    :param IT_MAX:
    :param DT:
    :param damp:
    :return: q_positions
    """

    JOINT_ID = 7
    q = pinocchio.neutral(model)
    i = 0

    while True:
        pinocchio.forwardKinematics(model, data, q)
        iMd = data.oMi[JOINT_ID].actInv(T)
        err = pinocchio.log(iMd).vector
        if norm(err) < eps:
            success = True
            break
        if i >= IT_MAX:
            success = False
            break
        J = pinocchio.computeJointJacobian(model, data, q, JOINT_ID)
        J = -np.dot(pinocchio.Jlog6(iMd.inverse()), J)
        v = -J.T.dot(solve(J.dot(J.T) + damp * np.eye(6), err))
        q = pinocchio.integrate(model, q, v * DT)
        if not i % 10:
            print(f"{i}: error = {err.T}")
        i += 1

    if success:
        print("Convergence achieved!")
    else:
        print("\n"
              "No convergence achieved!")
    return q


if __name__ == "__main__":
    import os
    import numpy as np
    from trac_ik import TracIK

    urdf_path = os.path.join(os.path.dirname(__file__), "..", "urdf", "franka_panda.urdf")
    iksolver = TracIK(base_link_name="panda_link0", tip_link_name="panda_link7", urdf_path=urdf_path)

    seed_jnt = np.array([-0.34906585, -1.57079633, -2.0943951, 0.52359878, 0.,
                         0.6981317, 0.])
    tgt_pos = np.array([0., -.1, 0.6])
    tgt_rotmat = np.array([[0.3536, -0.8660, 0.3536],
                           [0.6124, .5, -.6124],
                           [0.7071, 0., 0.7071]])

    result = iksolver.ik(tgt_pos=tgt_pos, tgt_rot=tgt_rotmat, seed_jnt_values=seed_jnt)
    print(result)
