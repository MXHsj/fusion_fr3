#! /usr/bin/env python3
# =================================================================
# file name:    cartesian_velocity_control_node.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import numpy as np
import panda_py

import rospy
from geometry_msgs.msg import TwistStamped

from controller.cartesian_velocity_controller import CartesianVelocityController

class CartesianVelocityControlNode():

  # TODO: 
  # - implement parameter updater

  def __init__(self, arm: panda_py.Panda, home:bool=False, frameEE:bool=True, rate:int=100):
    rospy.init_node('cartesian_velocity_control_node')

    self.twist_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # [linear, angular]
    self.cartesian_velocity_subscriber = rospy.Subscriber('fr3/Cartesian/velocity', TwistStamped, self.cartesian_velocity_cb)

    self.rate = rospy.Rate(rate)
    self.controller = CartesianVelocityController(arm=arm, 
                                                  home=home, 
                                                  frameEE=frameEE)

  def start_control(self):
    self.controller.start_controller()

  def stop_control(self):
    self.controller.stop_controller()

  def cartesian_velocity_cb(self, msg:TwistStamped) -> None:
    # TODO: use frame_id to determine space/body twist
    self.twist_cmd[0] = msg.twist.linear.x
    self.twist_cmd[1] = msg.twist.linear.y
    self.twist_cmd[2] = msg.twist.linear.z
    self.twist_cmd[3] = msg.twist.angular.x
    self.twist_cmd[4] = msg.twist.angular.y
    self.twist_cmd[5] = msg.twist.angular.z

  def reset_twist(self) -> None:
    self.twist_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # [linear, angular]

  def onUpdate(self) -> None:
    try:
      while not rospy.is_shutdown():
        self.controller.onUpdate(twist_cmd=self.twist_cmd)

        self.rate.sleep()

    finally:
      self.reset_twist()
      self.controller.onUpdate(twist_cmd=self.twist_cmd)  # stop motion
      self.controller.stop_controller()
      print('stopped Cartesian velocity controller')

  

if __name__ == '__main__':
  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  control_node = CartesianVelocityControlNode(arm=fr3, 
                                              home=True, 
                                              frameEE=True, 
                                              rate=100)

  control_node.onUpdate()