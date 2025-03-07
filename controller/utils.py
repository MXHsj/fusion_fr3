import numpy as np

VEL_COEFF = 0.8
CART_LIN_VEL_MAX = 3.0 * VEL_COEFF
CART_LIN_VEL_MIN = -3.0 * VEL_COEFF
CART_ANG_VEL_MAX = 2.5 * VEL_COEFF
CART_ANG_VEL_MIN = -2.5 * VEL_COEFF

Q_POS_MAX = [2.7437, 1.7837, 2.9009, -0.1518, 2.8065, 4.5169, 3.0159]


def avoid_cart_velocity_constraints():
  ...

def avoid_cart_pose_constraints():
  ...

def avoid_joint_pose_constraints():
  ...
