#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
import numpy as np
import pandas as pd
import math

def main():
    # Initialize the ROS node
    rospy.init_node('joint_data_publisher', anonymous=True)
    
    # Create a publisher for joint states
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # Load the robot chain from the URDF file
    my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])

    x = 0.03
    y = 0.13
    z = 0.04

    # Define the target position (x, y, z) and orientation ([0, -1, 0])
    target_position = [x, y, z]
    target_orientation = [0, -1, 0]

    # Calculate inverse kinematics
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")

    # Convert radians to degrees and round to 2 decimal points
    joint_angles_degrees = [round(math.degrees(angle) + 90, 2) for angle in ik.tolist()]

    # Extract only the desired joint angles (excluding the first and last elements)
    desired_joint_angles = joint_angles_degrees[1:-1]

    # Create a DataFrame row with formatted data
    joint_data_df = pd.DataFrame([desired_joint_angles + [x, y, z]], columns=[f'Joint {i+1}' for i in range(len(desired_joint_angles))] + ['x', 'y', 'z'])

    # Extract joint angles for publishing
    joint_angles = joint_data_df.iloc[0, :-3].values.tolist()  # Fix the indexing and column slicing
    
    # Create a JointState message
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = [f'joint_{i+1}' for i in range(len(joint_angles))]
    joint_state_msg.position = joint_angles
    
    # Publish the message
    rate = rospy.Rate(0.2)  # 0.2 Hz, i.e., once every 5 seconds
    while not rospy.is_shutdown():
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_pub.publish(joint_state_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
