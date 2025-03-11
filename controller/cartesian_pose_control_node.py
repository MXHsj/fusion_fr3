#! /usr/bin/env python3
# =================================================================
# file name:    cartesian_pose_control_node.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================

import sys

import numpy as np
import panda_py

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped

from fusion_fr3.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseFeedback, MoveToPoseResult

from controller.cartesian_pose_controller import CartesianPoseController
from controller.utils import PoseStampedToSE3

class CartesianPoseControlNode():
  # TODO: 
  # - implement parameter updater

  def __init__(self, arm:panda_py.Panda, rate:int=100) -> None:
    rospy.init_node('cartesian_pose_control_node')

    # self.controller = CartesianPoseController(arm=arm)

    self.pose_server = actionlib.SimpleActionServer(
      'fr3/Cartesian/pose',
      MoveToPoseAction,
      execute_cb=self.pose_execute_cb,
      auto_start=False
    )
    self.pose_server.start()
  
  def start_control(self):
    self.controller.start_controller()

  def stop_control(self):
    self.controller.stop_controller()

  def pose_execute_cb(self, goal:MoveToPoseGoal):
    success = True
    try:
      pose_goal_se3 = PoseStampedToSE3(goal.pose_goal)
      print(f'goal pose: \n{pose_goal_se3}')
      # self.controller.goto(pose_goal=pose_goal_se3)
      self.pose_server.set_succeeded(MoveToPoseResult(result=0 if success else 1))

    except Exception as e:
      result = MoveToPoseResult()
      result.success = False
      result.message = str(e)
      self.pose_server.set_aborted(result=result)

if __name__ == '__main__':
  default_ip = '172.16.0.2'

  if len(sys.argv) == 1:
    robot_ip = default_ip
  elif len(sys.argv) == 2:
    robot_ip = sys.argv[0]
  else:
    raise RuntimeError(f'Usage: python {sys.argv[0]} <robot-hostname>')
  # fr3 = panda_py.Panda(robot_ip)

  control_node = CartesianPoseControlNode(arm=fr3, 
                                          rate=100)

  rospy.spin()
  