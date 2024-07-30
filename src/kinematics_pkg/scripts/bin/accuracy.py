#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from ikpy.chain import Chain
import numpy as np
import pandas as pd
import math

def calculate_joint_angles(target_position):
    # Load the robot chain from the URDF file
    my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])
    
    # Define the target orientation (assuming [0, -1, 0] is fixed)
    target_orientation = [0, -1, 0]

    # Calculate inverse kinematics
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")

    # Convert radians to degrees and round to 2 decimal points
    joint_angles_degrees = [round(math.degrees(angle) + 90, 2) for angle in ik.tolist()]
    
    # Compute the end-effector position from the joint angles
    end_effector_position = my_chain.forward_kinematics(ik)[:3]  # Get only x, y, z
    
    return end_effector_position, joint_angles_degrees

def compute_end_effector_position(joint_angles):
    # Load the robot chain from the URDF file
    my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])

    # Convert joint angles back to radians
    joint_angles_radians = np.radians([angle - 90 for angle in joint_angles])
    
    # Calculate end-effector position from joint angles
    end_effector_position = my_chain.forward_kinematics(joint_angles_radians)[:3]  # Get only x, y, z
    return end_effector_position

def ik_callback(msg):
    target_position = [msg.x, msg.y, msg.z]
    
    # Calculate joint angles and end-effector position
    computed_position, joint_angles_degrees = calculate_joint_angles(target_position)
    computed_position = np.array(computed_position)  # Ensure this is a numpy array

    # Compute end-effector position from joint angles for accuracy comparison
    computed_effector_position = compute_end_effector_position(joint_angles_degrees)
    computed_effector_position = np.array(computed_effector_position)  # Ensure this is a numpy array
    
    # Calculate accuracy
    accuracy = np.linalg.norm(np.array(target_position) - computed_effector_position)
    
    rospy.loginfo(f"Target Position: {target_position}")
    rospy.loginfo(f"Computed End-Effector Position: {computed_effector_position}")
    rospy.loginfo(f"Accuracy (Euclidean distance): {accuracy:.4f}")

    # Create a JointState message
    joint_state_msg = JointState()
    joint_state_msg.header.stamp = rospy.Time.now()
    joint_state_msg.name = [f'joint_{i+1}' for i in range(len(joint_angles_degrees))]
    joint_state_msg.position = joint_angles_degrees
    
    # Publish the message
    joint_pub.publish(joint_state_msg)

def main():
    global joint_pub
    # Initialize the ROS node
    rospy.init_node('joint_data_publisher', anonymous=True)
    
    # Create a publisher for joint states
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # Create a subscriber for IK coordinates
    rospy.Subscriber('/ik_coordinates', Point, ik_callback)
    
    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

