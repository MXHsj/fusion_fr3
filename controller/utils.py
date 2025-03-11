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

Q_POS_MAX = [2.7437, 1.7837, 2.9009, -0.1518, 2.8065, 4.5169, 3.0159]


def avoid_cart_velocity_constraints():
  ...

def avoid_cart_pose_constraints():
  ...

def avoid_joint_pose_constraints():
  ...


def PoseStampedToSE3(poseIn:PoseStamped) -> SE3:
  poseOut = SE3()
  poseOut.t[0] = poseIn.pose.position.x
  poseOut.t[1] = poseIn.pose.position.y
  poseOut.t[2] = poseIn.pose.position.z
  quat = np.array([poseIn.pose.orientation.w,
                   poseIn.pose.orientation.x,
                   poseIn.pose.orientation.y,
                   poseIn.pose.orientation.z])
  R = UnitQuaternion(quat).SO3()
  poseOut.R = R
  return poseOut

def SE3ToPoseStamped(poseIn:SE3) -> PoseStamped:
  poseOut = PoseStamped()
  poseOut.pose.position.x = poseIn.t[0]
  poseOut.pose.position.y = poseIn.t[1]
  poseOut.pose.position.z = poseIn.t[2]
  poseOut.pose.orientation.w = SO3(poseIn.R).UnitQuaternion().data[0][0]
  poseOut.pose.orientation.x = SO3(poseIn.R).UnitQuaternion().data[0][1]
  poseOut.pose.orientation.y = SO3(poseIn.R).UnitQuaternion().data[0][2]
  poseOut.pose.orientation.z = SO3(poseIn.R).UnitQuaternion().data[0][3]

  return poseOut