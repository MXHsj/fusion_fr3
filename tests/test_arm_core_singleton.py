
import rospy
from robot.arm_core import Arm

if __name__ == '__main__':

  arm1 = Arm()
  print(f"Arm1 ID: {id(arm1)}")

  arm2 = Arm()
  print(f"Arm2 ID: {id(arm2)}")


