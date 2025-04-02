# =================================================================
# file name:    utils.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import numpy as np
from spatialmath import SE3, SO3, UnitQuaternion
from geometry_msgs.msg import PoseStamped

VEL_COEFF = 0.8
CART_LIN_VEL_MAX = 3.0 * VEL_COEFF
CART_LIN_VEL_MIN = -3.0 * VEL_COEFF
CART_ANG_VEL_MAX = 2.5 * VEL_COEFF
CART_ANG_VEL_MIN = -2.5 * VEL_COEFF

Q_POS_MAX = [2.7437, 1.7837, 2.9007, -0.1518, 2.8065, 4.5169, 3.0159]
Q_POS_MIN = [-2.7437, -1.7837, -2.9007, -3.0421, -2.8065, 0.5445, -3.0159]

Q_VEL_MAX = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]

def avoid_cart_velocity_constraints():
  ...

def avoid_cart_pose_constraints():
  ...

def avoid_joint_pose_constraints():
  ...

def damped_pseudo_inverse(J:np.ndarray, lambda_=0.01) -> np.ndarray:
  J_T = J.T
  _, n = J.shape
  damping = (lambda_ ** 2) * np.eye(n)
  return np.linalg.inv(J_T @ J + damping) @ J_T  # (JᵀJ + λ²I)⁻¹Jᵀ

def svd_damped_pseudo_inverse(J:np.ndarray, lambda_=0.01) -> np.ndarray:
  U, S, Vh = np.linalg.svd(J, full_matrices=False)
  S_damped = S / (S**2 + lambda_**2)    # apply damping to singular values
  return (Vh.T @ np.diag(S_damped) @ U.T)

def PoseStampedToSE3(pose_in:PoseStamped) -> SE3:
  pose_out_data = np.zeros((4,4), dtype=np.float64)
  pose_out_data[0, -1] = pose_in.pose.position.x
  pose_out_data[1, -1] = pose_in.pose.position.y
  pose_out_data[2, -1] = pose_in.pose.position.z
  quat = np.array([pose_in.pose.orientation.w,
                   pose_in.pose.orientation.x,
                   pose_in.pose.orientation.y,
                   pose_in.pose.orientation.z])
  R = UnitQuaternion(quat).SO3()
  pose_out_data[:3, :3] = R.data[0]
  pose_out = SE3(pose_out_data, check=False)
  
  return pose_out

def SE3ToPoseStamped(pose_in:SE3) -> PoseStamped:
  pose_out = PoseStamped()
  pose_out.pose.position.x = pose_in.t[0]
  pose_out.pose.position.y = pose_in.t[1]
  pose_out.pose.position.z = pose_in.t[2]

  pose_out.pose.orientation.w = UnitQuaternion(pose_in.R).data[0][0]
  pose_out.pose.orientation.x = UnitQuaternion(pose_in.R).data[0][1]
  pose_out.pose.orientation.y = UnitQuaternion(pose_in.R).data[0][2]
  pose_out.pose.orientation.z = UnitQuaternion(pose_in.R).data[0][3]

  return pose_out