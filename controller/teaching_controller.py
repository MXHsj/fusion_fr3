# =================================================================
# file name:    teaching_controller.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import panda_py
import logging
logging.basicConfig(level=logging.INFO)

POS_QUEUE_MAX = 5   # save up to N taught poses

class TeachingController():
  
  def __init__(self, arm:panda_py.Panda, home:bool=True) -> None:
    """
    Teaching mode: applied torque
    """
    self.arm = arm
    self.arm.teaching_mode(True)
    self.isActive = True
    self.pos_queue = []
    self.pos_count = 0
    self.rate = 50

  def start_controller(self):
    self.arm.teaching_mode(True)
    self.isActive = True

  def stop_controller(self):
    self.arm.teaching_mode(False)
    self.isActive(False)

  def get_recoded_pos(self) -> list:
    return self.pos_queue

  def record_pos(self) -> None:
    self.pos_queue.append(self.arm.q)
    self.pos_count += 1

  def replay_pos(self) -> None:
    if len(self.pos_queue):
      self.arm.move_to_joint_position(self.pos_queue)

  def reset_pos(self) -> None:
    self.pos_queue = []
    self.pos_count = 0

  def onUpdate(self):
    with self.arm.create_context(frequency=self.rate) as ctx:
      while ctx.ok():
        if self.pos_count < POS_QUEUE_MAX:
          input(f'press enter to record current position {self.pos_count}/{POS_QUEUE_MAX}')
          self.record_pos()
        else:
          input(f'press enter to execute recorded positions')
          self.replay_pos()
          self.reset_pos()
          

if __name__ == '__main__':
  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  controller = TeachingController(arm=fr3)
  controller.onUpdate()
