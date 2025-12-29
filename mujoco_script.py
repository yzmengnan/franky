import os
import mujoco
import time
import mujoco.viewer
import numpy as np



scene_path = os.path.join(os.path.dirname(__file__), 'mujoco_menagerie/franka_emika_panda', "scene.xml")

model = mujoco.MjModel.from_xml_path(scene_path)
data = mujoco.MjData(model)
home = np.array([-0.00323999, -0.78589636, 0.01407184, -2.36783182, -0.00823281, 1.57262848, 0.7871698])

if __name__ == "__main__":
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for i,qpos in enumerate(home):
            data.qpos[i] = qpos
        mujoco.mj_forward(model, data)
        mujoco.mj_kinematics(model,data)
        viewer.sync()
        while viewer.is_running():
            time.sleep(0.01)
            viewer.sync()

        viewer.close()
