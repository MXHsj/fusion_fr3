# =================================================================
# file name:    cartesian_impedance_controller.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================

import sys

import numpy as np
import panda_py
from panda_py import controllers
from spatialmath import SE3, SO3, UnitQuaternion

import logging
logging.basicConfig(level=logging.INFO)

default_impedance = np.array([[100.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 600
                              [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],    # 600
                              [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],    # 600
                              [0.0, 0.0, 0.0, 30.0, 0.0, 0.0],     # 30
                              [0.0, 0.0, 0.0, 0.0, 30.0, 0.0],     # 30
                              [0.0, 0.0, 0.0, 0.0, 0.0, 30.0]])    # 30

class CartesianImpedanceController():

  def __init__(self, arm:panda_py.Panda, home:bool=False) -> None:
    self.arm = arm
    if home:
      self.arm.move_to_start()

    self.default_impedance = np.array([[100.0, 0.0, 0.0, 0.0, 0.0, 0.0],    # 600
                                       [0.0, 100.0, 0.0, 0.0, 0.0, 0.0],    # 600
                                       [0.0, 0.0, 100.0, 0.0, 0.0, 0.0],    # 600
                                       [0.0, 0.0, 0.0, 30.0, 0.0, 0.0],     # 30
                                       [0.0, 0.0, 0.0, 0.0, 30.0, 0.0],     # 30
                                       [0.0, 0.0, 0.0, 0.0, 0.0, 30.0]])    # 30
    self.impedance = self.default_impedance.copy()
    self.nullspace_stiffness = 0.5
    self.filter_coeff = 1.0

    self.controller = controllers.CartesianImpedance(impedance=self.impedance, 
                                                     nullspace_stiffness=self.nullspace_stiffness,
                                                     filter_coeff=self.filter_coeff)
  
  def set_impedance(self, trans_imp:np.ndarray, rot_imp:np.ndarray) -> None:
    """
    Args:
      trans_imp : translational impedance
      rot_imp : rotational impedance
    """
    for t in range(3):
      self.impedance[t, t] = trans_imp[t]
    for r in range(3):
      self.impedance[r, r] = rot_imp[r]
    self.controller.set_impedance(self.impedance)

  def restore_impedance(self) -> None:
    self.controller.set_impedance(self.default_impedance)

  def set_nullspace_stiffness(self) -> None:
    ...

  def onUpdate(self, T0:SE3) -> None:
    x0 = T0.t
    q0 = SO3(T0.R).UnitQuaternion()
    self.controller.set_control(x0, q0)

if __name__ == '__main__':

  # TODO: move to unit test
  # TODO: check rt performance

  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  # ctrl = controllers.CartesianImpedance(impedance=default_impedance, filter_coeff=1.0)
  # fr3.start_controller(ctrl)

  T0 = SE3(fr3.get_pose(), check=False)
  controller = CartesianImpedanceController(arm=fr3, home=True)

  # x0 = fr3.get_position()
  # q0 = fr3.get_orientation()

  input('press enter to execute motion')
  with fr3.create_context(frequency=1e3) as ctx:
    while ctx.ok():
      
      controller.onUpdate(T0=T0)

      # ctrl.set_control(x0, q0)