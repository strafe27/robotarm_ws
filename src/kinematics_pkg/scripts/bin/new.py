#!/usr/bin/env python
# coding: utf-8
import rospy
from sensor_msgs.msg import JointState
import Arm_Lib

class Mover:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('dofbot_mover', anonymous=True)

        # Initialize the arm device
        self.arm = Arm_Lib.Arm_Device()

        # Initialize the ROS subscriber to 'joint_angles' topic
        self.subscriber = rospy.Subscriber('joint_states', JointState, self.joint_angles_callback)

    def joint_angles_callback(self, msg):
        """Callback function to handle incoming joint angles."""
        # Assuming the joint angles start from index 1 to 6 for joints 2 to 7
        if len(msg.position) > 6:
            # Extract angles for joints 2 to 7 (adjust indices as needed)
            angles = msg.position[1:7]

            # Convert degrees to servo units (if necessary)
            angles = [self.degrees_to_servo_units(angle) for angle in angles]

            # Set the joints on the arm
            self.arm.Arm_serial_servo_write6_array(angles, 1000)

        else:
            angles = msg.position[0:6]

            angles = [self.degrees_to_servo_units(angle) for angle in angles]

            # Set the joints on the arm
            self.arm.Arm_serial_servo_write6_array(angles, 1000)


    def degrees_to_servo_units(self, degrees):
        """Convert degrees to servo units if required."""
        # Implement conversion based on your specific servo calibration
        # For simplicity, we assume direct mapping
        return degrees

if __name__ == '__main__':
    target = Mover()
    rospy.spin()  # Keep the node running
