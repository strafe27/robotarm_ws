import rospy
from sensor_msgs.msg import JointState
import Arm_Lib

self.arm = Arm_Lib.Arm_Device()

gripper = Arm.Arm_serial_servo_read(6)

print(gripper)