# =================================================================
# file name:    cartesian_pose_controller.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import numpy as np
import panda_py
from spatialmath import SE3, SO3

from controller.cartesian_velocity_controller import CartesianVelocityController

Kp_pos_max = 1.0
Kp_ori_max = 1.0

class CartesianPoseController():
  
  def __init__(self, arm:panda_py.Panda, rate=100) -> None:
    self.Kp_pos = 0.65    # position Kp
    self.Kp_ori = 0.70    # orientation Kp
    self.pos_err_tol = 0.005
    self.ang_err_tol = 0.01
    self.isArrived = False
    self.rate = rate

    self.arm = arm
    self.controller = CartesianVelocityController(arm=arm, 
                                                  home=False, 
                                                  frameEE=False)  # use zero jacobian
    self.isActive = True
    
  def start_controller(self):
    self.controller.start_controller()
    self.isActive = True

  def stop_controller(self):
    self.controller.stop_controller()
    self.isActive = False

  def get_gain(self):
    return self.Kp_pos, self.Kp_ori

  def set_gain(self, Kp_pos:float, Kp_ori:float) -> None:
    if Kp_pos > Kp_ori_max or Kp_ori > Kp_ori_max:
      print(f'cannot set excessive gain, Kp max: {Kp_pos_max}, {Kp_ori_max}')
    else:
      self.Kp_pos = Kp_pos
      self.Kp_ori = Kp_ori

  def get_pos_tolerance(self) -> float:
    return self.pos_err_tol

  def get_ang_tolerance(self) -> float:
    return self.ang_err_tol

  def set_pos_tolerance(self, pos_err_tol:float) -> None:
    if pos_err_tol <= 0:
      print('invalid position tolerence')
    else:
      self.pos_err_tol = pos_err_tol

  def set_pos_tolerance(self, ang_err_tol:float) -> None:
    if ang_err_tol <= 0:
      print('invalid position tolerence')
    else:
      self.ang_err_tol = ang_err_tol

  def compute_cartesian_velocity(self, pose_goal: SE3):
    isArrived = False
    pose_curr = SE3(self.arm.get_pose())

    p_curr = pose_curr.t  # (3x1)
    p_goal = pose_goal.t  # (3x1)

    R_curr = pose_curr.R  
    R_goal = pose_goal.R

    pos_err = p_goal - p_curr  # (3,)

    R_err = R_goal @ R_curr.T  # Relative rotation
    rpy_err = SO3(R_err).rpy()

    vel_lin = self.Kp_pos * pos_err
    vel_ang = self.Kp_ori * rpy_err
    
    pos_err_mag = np.linalg.norm(pos_err)
    ang_err_mag = np.linalg.norm(rpy_err)

    if abs(pos_err_mag) < self.pos_err_tol and ang_err_mag < self.ang_err_tol:
      vel_lin = np.array([0.0, 0.0, 0.0])
      vel_ang = np.array([0.0, 0.0, 0.0])
      isArrived = True
      # print(f'pose goal reached!')

    twist_cmd = np.hstack((vel_lin, vel_ang))
    return twist_cmd, isArrived

  def goto(self, pose_goal: SE3):
    print('========== move to pose goal ==========')
    print(f'goal pose: \n{pose_goal}')
    with self.arm.create_context(frequency=self.rate) as ctx:
      while ctx.ok():
        twist_cmd, isArrived = self.compute_cartesian_velocity(pose_goal=pose_goal)
        self.controller.onUpdate(twist_cmd=twist_cmd)
        if isArrived:
          print('arrived!')
          break


if __name__ == '__main__':

  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  controller = CartesianPoseController(arm=fr3)

  T_d1 = np.array([[1.0, 0.0, 0.0, 0.4],
                  [0.0, -1.0, 0.0, -0.2],
                  [0.0, 0.0, -1.0, 0.4],
                  [0.0, 0.0, 0.0, 1.0]])
  T_d1 = SE3(T_d1, check=True)

  controller.goto(pose_goal=T_d1)

  T_d2 = np.array([[1.0, 0.0, 0.0, 0.35],
                  [0.0, -1.0, 0.0, 0.0],
                  [0.0, 0.0, -1.0, 0.5],
                  [0.0, 0.0, 0.0, 1.0]])
  T_d2 = SE3(T_d2, check=True)

  input('press enter to execute motion')
  controller.goto(pose_goal=T_d2)
  