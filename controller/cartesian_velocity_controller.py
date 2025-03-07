# =================================================================
# file name:    cartesian_velocity_controller.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import numpy as np
import panda_py
from panda_py import controllers
from spatialmath import SE3
import logging

logging.basicConfig(level=logging.INFO)

class CartesianVelocityController():

  def __init__(self, arm: panda_py.Panda, home:bool=True, frameEE:bool=True):
    self.frameEE = frameEE
    self.arm = arm
    if home:
      self.arm.move_to_start()

    self.controller = controllers.IntegratedVelocity()
    self.isActive = True
    self.start_controller()
    print('initialization complete.')
    print(f'current ee pose:\n{SE3(self.arm.get_pose(), check=False)}')

  def start_controller(self):
    self.arm.start_controller(self.controller)
    self.isActive = True

  def stop_controller(self):
    self.arm.stop_controller()
    self.isActive = False

  def get_controller_status(self):
    return self.isActive

  def get_body_jacobian(self) -> np.ndarray:
    model = self.arm.get_model()
    J_b = model.body_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                              q=self.arm.q,
                              EE_T_K=self.arm.get_state().EE_T_K, 
                              F_T_EE=self.arm.get_state().F_T_EE)
    J_b = np.array(J_b).reshape(7, 6).T
    return J_b

  def get_zero_jacobian(self) -> np.ndarray:
    model = self.arm.get_model()
    J_z = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                              q=self.arm.q,
                              EE_T_K=self.arm.get_state().EE_T_K, 
                              F_T_EE=self.arm.get_state().F_T_EE)
    J_z = np.array(J_z).reshape(7, 6).T
    return J_z

  def set_stiffness(self):
    ...

  def set_damping(self):
    ...

  def onUpdate(self, twist_cmd: np.ndarray) -> None:
    if self.frameEE:
      J = self.get_body_jacobian()
    else:
      J = self.get_zero_jacobian()
    J_pinv = np.linalg.pinv(J)
    dq = J_pinv @ twist_cmd
    self.controller.set_control(dq)

if __name__ == '__main__':
  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  controller = CartesianVelocityController(arm=fr3, 
                                           home=True, 
                                           frameEE=True)
  
  lin_velocity = np.array([0.0, 0.0, 0.03])
  ang_velocity = np.array([0.0, 0.0, 0.0])
  cart_velocity = np.concatenate((lin_velocity, ang_velocity))
  
  input('press enter to execute motion')
  with fr3.create_context(frequency=100) as ctx:
    while ctx.ok():
      controller.onUpdate(twist_cmd=cart_velocity)
