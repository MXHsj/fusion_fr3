#! /usr/bin/env python3
# =================================================================
# file name:    joint_velocity_control_node.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys
import threading

import numpy as np
import panda_py
from panda_py import controllers

import rospy
from std_msgs.msg import Float64MultiArray

class JointVelocityControlNode():
  
  def __init__(self, arm:panda_py.Panda,
                     rate:int=100,
                     stop_event:threading.Event=None):

    self.arm = arm

    self.joint_vel_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # q1-q7
    self.joint_vel_cmd_last = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    self.last_msg_time = None
    self.msg_timeout = 1.0    #  [sec]
    self.joint_velocity_subscriber = rospy.Subscriber('fr3/controller/joint/velocity', Float64MultiArray, self.joint_velocity_cb)

    self.rate = rospy.Rate(rate)

    self.controller = controllers.IntegratedVelocity()
    self.isActive = True
    self.start_controller()

    self.stop_event = stop_event

  def start_controller(self):
    self.arm.start_controller(self.controller)
    self.isActive = True

  def stop_controller(self):
    self.arm.stop_controller()
    self.isActive = False

  def joint_velocity_cb(self, msg:Float64MultiArray):
    assert len(msg.data) == 7
    self.joint_vel_cmd[0] = msg.data[0]
    self.joint_vel_cmd[1] = msg.data[1]
    self.joint_vel_cmd[2] = msg.data[2]
    self.joint_vel_cmd[3] = msg.data[3]
    self.joint_vel_cmd[4] = msg.data[4]
    self.joint_vel_cmd[5] = msg.data[5]
    self.joint_vel_cmd[6] = msg.data[6]
    self.joint_vel_cmd_last = self.joint_vel_cmd.copy()
    self.last_msg_time = rospy.get_time()

  def reset_joint_vel(self) -> None:
    self.joint_vel_cmd = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])   # q1-q7

  def onUpdate(self) -> None:
    try:
      while not rospy.is_shutdown():
        current_time = rospy.get_time()
        elapsed = current_time - self.last_msg_time if self.last_msg_time else float('inf')
        if elapsed >= self.msg_timeout:
          self.reset_joint_vel()
        self.controller.set_control(self.joint_vel_cmd)

        if self.stop_event is not None:
          if self.stop_event.is_set():
            break
        self.rate.sleep()

    finally:
      self.reset_joint_vel()
      self.controller.set_control(self.joint_vel_cmd)  # stop motion
      self.stop_controller()
      print('stopped joint velocity controller')

  
if __name__ == '__main__':
  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  fr3 = panda_py.Panda(robot_ip)

  rospy.init_node('joint_velocity_control_node')
  control_node = JointVelocityControlNode(arm=fr3, 
                                          rate=100)

  control_node.onUpdate()