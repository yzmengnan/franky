from franky import *
import numpy as np
from scipy.spatial.transform import Rotation
from utils.geo import genTrans_se
import math, time
from modern_robotics import *

from utils.img_record import ImageGrap

robot = Robot("172.16.0.3")  # Replace this with your robot's IP
ig = ImageGrap()
ig.show()

robot.relative_dynamics_factor = 0.1
robot.translation_acceleration_limit.set(1)
robot.translation_velocity_limit.set(1)
robot.rotation_acceleration_limit.set(1)
robot.rotation_velocity_limit.set(1)

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

home_q3 = [-0.35863276, 0.35862066, 0.01463398, -2.17422068, 0.83391615, 2.03485268
    , 0.64526337]

home2 = JointMotion(home_q2)
home3 = JointMotion(home_q3)
robot.move(home2)
# robot.move(home3)
time.sleep(0.5)

home_start_rotation = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).T
home_start_rotation = home_start_rotation @ MatrixExp3(VecToso3(np.array([0, 0, -np.pi / 4])))
quan_homes = Rotation.from_matrix(home_start_rotation).as_quat()

x_offset = 0.45
c_offset = 0.1
c_offset2 = 0.03

motion_c_homes = CartesianMotion(
    Affine([x_offset, c_offset2, c_offset], quan_homes), ReferenceType.Absolute
)
robot.move(motion_c_homes)
print("push any key to continue")
input()

trans_se_list = genTrans_se(
    distance=-0.0,
    steps=20,
    x_offset=x_offset,
    theta=np.deg2rad(35),
    start=-math.pi * 1.5,
    end=-math.pi * 0.5,
)

T_ce = RpToTrans(MatrixExp3(VecToso3(np.array([0, 0, -np.pi / 4]))), np.array([0, -c_offset2, -c_offset]))
t = trans_se_list[0] @ T_ce
print(t)

cp_1 = CartesianMotion(
    Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat()), ReferenceType.Absolute
)
robot.move(cp_1)
print("push any key to continue")
input()

# robot.move(home2)

c_state = robot.current_cartesian_state
print(c_state.pose.end_effector_pose.quaternion)
rot = Rotation.from_quat(c_state.pose.end_effector_pose.quaternion)
print(rot)

c = Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat())
print(c)

motion_list = []

for t in trans_se_list:
    t = t @ T_ce
    cp = CartesianMotion(
        Affine(t[:3, -1], Rotation.from_matrix(t[:3, :3]).as_quat()),
        ReferenceType.Absolute,
    )
    motion_list.append(cp)


def motion(motion_list, sleep):
    for i, motion in enumerate(motion_list):
        robot.move(motion)
        time.sleep(sleep)
        if i == 0:
            # print(robot.current_joint_state.position)
            time.sleep(sleep * 4)


def motion_with_img_grap(motion_list, sleep, ig: ImageGrap, img_path):
    for i, motion in enumerate(motion_list):
        robot.move(motion)
        time.sleep(sleep)
        if i == 0:
            time.sleep(sleep * 4)
        ig.save_img(img_path, str(i))


def motion_reverse(motion_list, sleep):
    for motion in reversed(motion_list):
        robot.move(motion)
        time.sleep(sleep)


if __name__ == "__main__":
    counts = 3
    sleep = 0.5
    img_path = 'img'
    while counts > 0:
        # motion(motion_list, sleep=sleep)
        motion_with_img_grap(motion_list,sleep,ig,img_path)
        motion_reverse(motion_list, sleep=sleep)
        counts -= 1
    exit(0)

# m_jp1 = JointMotion([-0.3, 0.1, 0.3, -1.4, 0.1, 1.8, 0.7])
# print(m_jp1)

# Move the robot 20cm along the relative X-axis of its end-effector
# motion = CartesianMotion(Affine([-0.01, 0.0, 0.1]), ReferenceType.Absolute)
# robot.move(motion)
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
