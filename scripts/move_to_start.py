# =================================================================
# file name:    move_to_start.py
# description:  move fr3 to default start pose
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import panda_py
from spatialmath import SE3

if __name__ == '__main__':

  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')

  panda = panda_py.Panda(robot_ip)
  panda.move_to_start()
  
  print('========== moved to start pose ==========')
  print(SE3(panda.get_pose()))