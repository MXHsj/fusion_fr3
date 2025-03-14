import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray


if __name__ == '__main__':
  rospy.init_node('test_joint_velocity_control', anonymous=True)

  joint_vel_pub = rospy.Publisher('fr3/controller/joint/velocity', Float64MultiArray, queue_size=1)

  rate = rospy.Rate(50)
  joint_vel_cmd = np.array([0.0, -0.01, 0.0, 0.0, 0.0, 0.0, 0.0])

  while not rospy.is_shutdown():
    joint_vel_cmd_msg = Float64MultiArray()
    joint_vel_cmd_msg.data = joint_vel_cmd
    joint_vel_pub.publish(joint_vel_cmd_msg)
    rate.sleep()
