# =================================================================
# file name:    cartesian_velocity_control_node.py
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

import rospy
from geometry_msgs.msg import TwistStamped

class CartesianVelocityControlNode():

  def __init__(self):
    rospy.init_node('cartesian_velocity_control_node')
  
    self.cartesian_velocity_subscriber = rospy.Subscriber('fr3/Cartesian/velocity', TwistStamped, self.cartesian_velocity_cb)

  def cartesian_velocity_cb(self):
    ...

  

if __name__ == '__main__':
  ...