# =================================================================
# file name:    arm_core.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys

import rospy
import actionlib
import numpy as np
import panda_py
from panda_py import controllers
from spatialmath import SE3, UnitQuaternion

from std_srvs.srv import Empty, SetBool, SetBoolResponse
from geometry_msgs.msg import PoseStamped
from fusion_fr3.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseFeedback, MoveToPoseResult

from controller.cartesian_velocity_control_node import CartesianVelocityControlNode
from controller.cartesian_pose_controller import CartesianPoseController
from controller.cartesian_impedance_control_node import CartesianImpedanceControlNode
from controller.teaching_control_node import TeachingControlNode
from controller.cartesian_pose_server import CartesianPoseServer
from controller.utils import PoseStampedToSE3

default_ip = '172.16.0.2'
ENFORCE_RT = panda_py.libfranka.RealtimeConfig.kEnforce
IGNORE_RT = panda_py.libfranka.RealtimeConfig.kIgnore

controllers = {
  "cartesian_velocity"    : CartesianVelocityControlNode,
  "cartesian_pose"        : CartesianPoseController,
  "cartesian_impedance"   : CartesianImpedanceControlNode,
  "teaching"              : TeachingControlNode
}

class Arm():

  # TODO:
  # - implement controller switch service

  def __init__(self,ip:str=default_ip,
                    default_controller:str="cartesian_velocity",
                    rt:bool=False) -> None:

    rospy.init_node('arm_core', anonymous=True)

    # ========== configure arm ==========
    self.arm_ip = ip
    if rt:
      self.rt_config = ENFORCE_RT
      print('enfore real-time kernel')
    else:
      self.rt_config = IGNORE_RT
      print('ignore real-time kernel')
    # self.arm = panda_py.Panda(self.arm_ip, realtime_config=self.rt_config)
    # ===================================

    # ========== start default controller ==========
    self.default_controller = default_controller
    self.current_controller = default_controller
    print(f'using controller: {default_controller}')
    # self.controller = controllers[default_controller](arm=self.arm)
    # ==============================================

    # ========== move_to_pose server ==========
    self.pose_server = actionlib.SimpleActionServer(
      'fr3/Cartesian/pose',
      MoveToPoseAction,
      execute_cb=self.pose_execute_cb,
      auto_start=False
    )
    self.pose_server.start()
    # =========================================

  def get_rt_config(self):
    return self.rt_config

  def switch_controller(self, new_controller_name:str):
    # stop current controller
    # start new controller 
    print(f'switching from {self.current_controller} to {new_controller_name} controller')
    # self.controller.stop_control()
    # self.controller = controllers[new_controller_name](arm=self.arm)
  
  def pose_execute_cb(self, goal:MoveToPoseGoal):
    success = True
    try:
      self.switch_controller('cartesian_pose')
      self.current_controller = 'cartesian_pose'
      pose_goal_se3 = PoseStampedToSE3(goal.pose_goal)
      print(f'goal pose: \n{pose_goal_se3}')
      
      # self.controller.goto(pose_goal=pose_goal_se3)
      
      self.switch_controller(self.default_controller)
      
      self.pose_server.set_succeeded(MoveToPoseResult(result=0 if success else 1))

    except Exception as e:
      result = MoveToPoseResult()
      result.success = False
      result.message = str(e)
      self.pose_server.set_aborted(result=result)

  def run(self):
    while not rospy.is_shutdown():
      pass



if __name__ == '__main__':

  arm = Arm()  
  arm.run()