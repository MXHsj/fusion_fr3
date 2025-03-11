#!/usr/bin/env python3
import numpy as np
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from spatialmath import SE3, SO3, UnitQuaternion

from fusion_fr3.msg import MoveToPoseAction, MoveToPoseGoal

from controller.utils import SE3ToPoseStamped

T_d1 = np.array([[1.0, 0.0, 0.0, 0.4],
                 [0.0, -1.0, 0.0, -0.2],
                 [0.0, 0.0, -1.0, 0.4],
                 [0.0, 0.0, 0.0, 1.0]])
T_d1 = SE3(T_d1, check=True)

T_d2 = np.array([[1.0, 0.0, 0.0, 0.35],
                 [0.0, -1.0, 0.0, 0.0],
                 [0.0, 0.0, -1.0, 0.5],
                 [0.0, 0.0, 0.0, 1.0]])
T_d2 = SE3(T_d2, check=True)

def move_to_pose_client(T_d:SE3):
  client = actionlib.SimpleActionClient('fr3/Cartesian/pose', MoveToPoseAction)
  client.wait_for_server()

  goal = MoveToPoseGoal()
  goal.pose_goal = SE3ToPoseStamped(T_d)
  print(f'sending goal pose: {goal.pose_goal}')

  client.send_goal(goal)
  result = client.wait_for_result()

  return result

if __name__ == '__main__':
  rospy.init_node('test_pose_action_client')
  
  result = move_to_pose_client(T_d=T_d1)
  print(f'succeeded: {result}')

  result = move_to_pose_client(T_d=T_d2)
  print(f'succeeded: {result}')





