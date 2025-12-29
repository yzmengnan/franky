from franky import *
import numpy as np
from scipy.spatial.transform import Rotation
from utils.geo import genTrans_se
import math, time

robot = Robot("172.16.0.3")  # Replace this with your robot's IP

robot.relative_dynamics_factor = 0.1
robot.translation_acceleration_limit.set(2)
robot.translation_velocity_limit.set(2)
robot.rotation_acceleration_limit.set(2)
robot.rotation_velocity_limit.set(2.5)

robot.recover_from_errors()
state = robot.current_joint_state
print(state.position)

home = JointMotion(
    [
        -0.00323999,
        -0.78589636,
        0.01407184,
        -2.36783182,
        -0.00823281,
        1.57262848,
        0.7871698,
    ]
)

home_q = [
    -0.00323999,
    -0.78589636,
    0.01407184,
    -2.36783182,
    -0.00823281,
    1.57262848,
    0.7871698,
]
home_q2 = [
    -0.01354345,
    0.40974365,
    0.01427194,
    -2.03035406,
    -0.00883915,
    2.44512407,
    0.79156684,
]

home2 = JointMotion(home_q2)
robot.move(home2)
time.sleep(0.5)

# state = robot.state
#
# cartesian_state = robot.current_cartesian_state
# robot_pose = cartesian_state.pose
# ee_pose = robot_pose.end_effector_pose

# print(ee_pose.translation)
#
# m_cp1 = CartesianMotion(
#     Affine([0.3, 0, 0.34], ee_pose.quaternion), ReferenceType.Absolute
# )

# robot.move(m_cp1)

# print(robot.current_cartesian_state.pose.end_effector_pose.translation)
#
# ee_pose_kin = robot.model.pose(Frame.EndEffector, home_q, Affine(), Affine())
#
# print(ee_pose_kin)

# rotation = Rotation.from_matrix(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).T)
# print(rotation)
# quan_star = rotation.as_quat()
# print(quan_star)

# m_cp2 = CartesianMotion(Affine([0.6, 0, 0.1], quan_star), ReferenceType.Absolute)
# robot.move(m_cp2)
# home2 = robot.current_joint_state.position
# print(home2)

trans_se_list = genTrans_se(
    distance=0.1,
    steps=20,
    x_offset=0.6,
    theta=np.deg2rad(70),
    start=-math.pi * 1.5,
    end=-math.pi * 0.5,
)
t = trans_se_list[0]
print(t)

cp_1 = CartesianMotion(
    Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat()), ReferenceType.Absolute
)
robot.move(cp_1)
time.sleep(0.5)

c_state = robot.current_cartesian_state
print(c_state.pose.end_effector_pose.quaternion)
rot = Rotation.from_quat(c_state.pose.end_effector_pose.quaternion)
print(rot)


c = Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat())
print(c)


motion_list = []

for t in trans_se_list:
    cp = CartesianMotion(
        Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat()),
        ReferenceType.Absolute,
    )
    motion_list.append(cp)


def motion(motion_list, sleep):
    for motion in motion_list:
        robot.move(motion)
        time.sleep(sleep)


def motion_reverse(motion_list, sleep):
    for motion in reversed(motion_list):
        robot.move(motion)
        time.sleep(sleep)


if __name__ == "__main__":
    counts = 3
    sleep = 0.5
    while counts > 0:
        motion(motion_list, sleep=sleep)
        motion_reverse(motion_list, sleep=sleep)
        counts -= 1
    exit(0)

# m_jp1 = JointMotion([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7])
# print(m_jp1)

# Move the robot 20cm along the relative X-axis of its end-effector
# motion = CartesianMotion(Affine([-0.01, 0.0, 0.1]), ReferenceType.Absolute)
# robot.move(motion)
