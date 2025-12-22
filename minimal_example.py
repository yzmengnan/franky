import os
import time
import mujoco.viewer
import numpy as np
from utils.ik import ik_trac, IK_Traj
import logging
from utils.kinematics import *
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--steps', type=int, default=2000)
parser.add_argument('--distance', type=float, default=.3)
parser.add_argument('--camera_distance', type=float, default=.1)
parser.add_argument('--height', type=float, default=.2)
parser.add_argument('--angle', type=float, default=65, help='angle in degrees')
parser.add_argument('--speed', type=float, default=.0)
args = parser.parse_args()

logging.basicConfig(level=logging.INFO, format='[%(asctime)s]-[%(levelname)s]-%(message)s')

path = 'mujoco_menagerie'
xml_path = os.path.join(path, 'franka_emika_panda', 'scene.xml')

# load model
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# print nbody
logging.info(f"{[model.body(i).name for i in range(model.nbody)]}")

# set ee
EE_SITE_NAME = 'hand'
ee_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, EE_SITE_NAME)
print(ee_id)

urdf_path = os.path.join(os.path.dirname(__file__), "urdf", "franka_panda.urdf")

seed_jnt = np.zeros(7)

solver = IK_Traj(urdf_path=urdf_path, target_frame='panda_hand', base_frame='panda_link0')

t_te = get_t_te(args.angle, args.camera_distance, args.steps)
t_se = get_t_se(t_te, args.distance, height=args.height)

traj = solver.gen_trajectory(start_qpos=seed_jnt, t_m=t_se)

real_time = False
cw_or_ccw = True
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        if not real_time:
            for step in range(len(traj) - 1):
                if not cw_or_ccw:
                    step = -1 * step - 2
                    if step + len(traj) < 0:
                        continue
                for i, joint_id in enumerate(np.arange(0, 7)):
                    data.qpos[model.jnt_qposadr[joint_id]] = traj[step][i]
                mujoco.mj_kinematics(model, data)
                mujoco.mj_forward(model, data)
                viewer.sync()
                if viewer.is_running():
                    time.sleep(args.speed)
                    # logging.info(data.xpos[model.body('hand').id])
            cw_or_ccw = False if cw_or_ccw else True

        else:
            # for step in range(len(traj)):
            for step in range(2000):
                for i, joint_id in enumerate(np.arange(0, 7)):
                    t_m = t_se[step]
                    res = solver.gen_qpos_realtime(seed_jnt, t_m=t_m)
                    if res is None:
                        logging.error("ik solver failed")
                        exit(-3)
                    seed_jnt = res
                    # data.qpos[model.jnt_qposadr[joint_id]] = traj[step][i]
                    data.qpos[model.jnt_qposadr[joint_id]] = seed_jnt[i]
                mujoco.mj_kinematics(model, data)
                mujoco.mj_forward(model, data)
                # mujoco.mj_step(model, data)
                # print(data.xpos[7])
                viewer.sync()
                # if step == 0:
                #     time.sleep(3)
                # else:
                if viewer.is_running():
                    # time.sleep(0.01)
                    logging.info(data.xpos[model.body('hand').id])

            for step in range(2000):
                for i, joint_id in enumerate(np.arange(0, 7)):
                    t_m = t_se[-step - 1]
                    res = solver.gen_qpos_realtime(seed_jnt, t_m=t_m)
                    if res is None:
                        logging.error("ik solver failed")
                        exit(-3)
                    seed_jnt = res
                    # data.qpos[model.jnt_qposadr[joint_id]] = traj[step][i]
                    data.qpos[model.jnt_qposadr[joint_id]] = seed_jnt[i]
                mujoco.mj_kinematics(model, data)
                mujoco.mj_forward(model, data)
                # mujoco.mj_step(model, data)
                # print(data.xpos[7])
                viewer.sync()
                # if step == 0:
                #     time.sleep(3)
                # else:
                if viewer.is_running():
                    # time.sleep(0.01)
                    logging.info(data.xpos[model.body('hand').id])

    viewer.close()
