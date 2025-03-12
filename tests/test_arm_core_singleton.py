from robot.arm_core import Arm

if __name__ == '__main__':
  arm1 = Arm()
  arm2 = Arm()

  print(f"Arm 1 ID: {id(arm1)}")
  print(f"Arm 2 ID: {id(arm2)}")

  assert arm1 is arm2, "singleton test failed"

