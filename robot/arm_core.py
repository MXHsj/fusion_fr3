#! /usr/bin/env python3
# =================================================================
# file name:    arm_core.py
# description:  
# author:       Xihan Ma
# date:         2025-03-05
# =================================================================
import sys
from copy import copy
import threading

import rospy
import actionlib
import numpy as np
import panda_py
from panda_py import controllers
from spatialmath import SE3, UnitQuaternion

from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import WrenchStamped
from fusion_fr3.msg import MoveToPoseAction, MoveToPoseGoal, MoveToPoseFeedback, MoveToPoseResult

from controller.cartesian_velocity_control_node import CartesianVelocityControlNode
from controller.cartesian_pose_controller import CartesianPoseController
from controller.cartesian_impedance_control_node import CartesianImpedanceControlNode
from controller.teaching_control_node import TeachingControlNode
from controller.joint_velocity_control_node import JointVelocityControlNode
from controller.utils import PoseStampedToSE3

default_ip = '172.16.0.2'
ENFORCE_RT = panda_py.libfranka.RealtimeConfig.kEnforce
IGNORE_RT = panda_py.libfranka.RealtimeConfig.kIgnore

controllers = {
  "cartesian_velocity"    : CartesianVelocityControlNode,
  "cartesian_pose"        : CartesianPoseController,
  "cartesian_impedance"   : CartesianImpedanceControlNode,
  "teaching"              : TeachingControlNode,
  "joint_velocity"        : JointVelocityControlNode
}

class Arm():
  # TODO: fix rt-kernel threading issue

  _instance = None
  _initialized = False

  def __new__(cls, *args, **kwargs):
    if cls._instance is None:
      cls._instance = super(Arm, cls).__new__(cls)
    return cls._instance

  def __init__(self,ip:str=default_ip,
                    default_controller:str="cartesian_velocity",
                    rt:bool=False,
                    rate:int=200) -> None:

    # ensure singleton in a single process 
    if self._initialized:
      print('instance already exists, skip initialization')
      return

    self._initialized = True

    rospy.init_node('arm_core', anonymous=True)

    # ========== configure arm ==========
    self.arm_ip = ip
    if rt:
      self.rt_config = ENFORCE_RT
      print('enfore real-time kernel')
    else:
      self.rt_config = IGNORE_RT
      print('ignore real-time kernel')
    self.arm = panda_py.Panda(self.arm_ip, realtime_config=self.rt_config)
    # ===================================

    # ========== start default controller ==========
    self.stop_event = threading.Event()
    self.default_controller = default_controller
    self.current_controller = copy(self.default_controller)
    print(f'using controller: {default_controller}')
    self.controller = controllers[default_controller](arm=self.arm, stop_event=self.stop_event)
    # ==============================================

    # ========== ROS actions ==========
    self.pose_server = actionlib.SimpleActionServer(
      'fr3/action/move_to_pose',
      MoveToPoseAction,
      execute_cb=self.pose_execute_cb,
      auto_start=False
    )
    self.pose_server.start()
    # =================================

    # ========== ROS services ==========
    rospy.Service('fr3/service/move_to_start', Trigger, self.move_to_start)
    # ==================================

    # ========== ROS topics ==========
    self.state_publisher_rate = 100
    self.F_ext_publisher = rospy.Publisher('fr3/state/F_ext', WrenchStamped, queue_size=1)
    self.O_T_EE_publisher = rospy.Publisher('fr3/state/O_T_EE', Float64MultiArray, queue_size=1)
    self.q_publisher = rospy.Publisher('fr3/state/q', Float64MultiArray, queue_size=1)
    self.J_body_publisher = rospy.Publisher('fr3/state/body_jacobian', Float64MultiArray, queue_size=1)
    self.J_zero_publisher = rospy.Publisher('fr3/state/zero_jacobian', Float64MultiArray, queue_size=1)
    self.state_timer = rospy.Timer(rospy.Duration(1/self.state_publisher_rate), self.arm_state_publisher)
    # ================================

    # ========== start background thread ==========
    self.rate = rospy.Rate(rate)
    self.run_thread = threading.Thread(target=self.run, daemon=True)
    self.run_thread.start()
    # =============================================

  def get_rt_config(self):
    return self.rt_config

  def set_ee(self):
    ...

  def set_load(self):
    ...

  def switch_controller(self, new_controller_name:str) -> None:
    rospy.loginfo(f'switching from {self.current_controller} to {new_controller_name} controller')
    self.stop_event.set()
    if self.run_thread.is_alive():
      self.run_thread.join(timeout=1)
    self.stop_event.clear()

    self.controller.stop_controller()
    self.controller = controllers[new_controller_name](arm=self.arm, stop_event=self.stop_event)
    self.current_controller = new_controller_name

    self.run_thread = threading.Thread(target=self.run, daemon=True)
    self.run_thread.start()
  
  def move_to_start(self, req):
    self.arm.move_to_start()
    success = True
    message = 'moved to start position'
    return TriggerResponse(success, message)

  def pose_execute_cb(self, goal:MoveToPoseGoal) -> None:
    success = True
    try:
      self.switch_controller('cartesian_pose')

      pose_goal_se3 = PoseStampedToSE3(goal.pose_goal)
      # print(f'goal pose: \n{pose_goal_se3}')
      self.controller.goto(pose_goal=pose_goal_se3)
      
      self.pose_server.set_succeeded(MoveToPoseResult(success=0 if success else 1))
      self.switch_controller(self.default_controller)
      
    except Exception as e:
      result = MoveToPoseResult()
      result.success = False
      result.message = str(e)
      rospy.logerr('move to pose server error: '+str(e))
      self.pose_server.set_aborted(result=result)

  def arm_state_publisher(self, event=None) -> None:
    # ===== ee pose =====
    O_T_EE_msg = Float64MultiArray(data=self.arm.get_pose().flatten())
    self.O_T_EE_publisher.publish(O_T_EE_msg)
    # ===== external wrench w.r.t ee frame =====
    F_ext_msg = WrenchStamped()
    F_ext_msg.header.frame_id = 'ee'
    F_ext_msg.header.stamp = rospy.Time().now()
    F_ext_msg.wrench.force.x = self.arm.get_state().K_F_ext_hat_K[0]
    F_ext_msg.wrench.force.y = self.arm.get_state().K_F_ext_hat_K[1]
    F_ext_msg.wrench.force.z = self.arm.get_state().K_F_ext_hat_K[2]
    F_ext_msg.wrench.torque.x = self.arm.get_state().K_F_ext_hat_K[3]
    F_ext_msg.wrench.torque.y = self.arm.get_state().K_F_ext_hat_K[4]
    F_ext_msg.wrench.torque.z = self.arm.get_state().K_F_ext_hat_K[5]
    self.F_ext_publisher.publish(F_ext_msg)
    # ===== joint angle =====
    q_msg = Float64MultiArray(data=self.arm.get_state().q)
    self.q_publisher.publish(q_msg)
    # ===== jacobian =====
    model = self.arm.get_model()
    J_b = model.body_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                              q=self.arm.q,
                              EE_T_K=self.arm.get_state().EE_T_K, 
                              F_T_EE=self.arm.get_state().F_T_EE)
    J_z = model.zero_jacobian(frame=panda_py.libfranka.Frame.kEndEffector, 
                              q=self.arm.q,
                              EE_T_K=self.arm.get_state().EE_T_K, 
                              F_T_EE=self.arm.get_state().F_T_EE)
    J_b_msg = Float64MultiArray(data=np.array(J_b).flatten())
    J_z_msg = Float64MultiArray(data=np.array(J_z).flatten())
    self.J_body_publisher.publish(J_b_msg)
    self.J_zero_publisher.publish(J_z_msg)

  def run(self) -> None:
    self.controller.onUpdate()
    # pass

if __name__ == '__main__':

  default_controller = rospy.get_param('~default_controller', 'cartesian_velocity')

  arm = Arm(default_controller=default_controller,
            rt=False,
            rate=200)
  rospy.spin()