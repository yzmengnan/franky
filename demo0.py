import math
from scipy.spatial.transform import Rotation
from franky import Affine
import numpy as np

z_translation = Affine(np.array([0.0, 0.0, 0.5]))

quat = Rotation.from_euler("xyz", [0, 0, math.pi / 2]).as_quat()
z_rotation = Affine(np.array([0.0, 0.0, 0.0]), quat)

combined_transformation = z_translation * z_rotation
print(combined_transformation)
