#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from ikpy.chain import Chain
import numpy as np
import pandas as pd
import math

def calculate_ik(my_chain, target_position, target_orientation):
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z")
    adjusted_angles = [angle + math.radians(90) for angle in ik]
    adjusted_angles_degrees = list(map(lambda r: round(math.degrees(r), 5), adjusted_angles))
    computed_position = my_chain.forward_kinematics(ik)
    return adjusted_angles_degrees, computed_position[:3, 3]

def calculate_accuracy(input_position, computed_position):
    error = np.linalg.norm(np.array(input_position) - np.array(computed_position))
    accuracy = 100 * (1 - error / np.linalg.norm(np.array(input_position)))
    return accuracy

def callback(data):
    x, y, z = data.x, data.y, data.z
    target_position = [x, y, z]
    target_orientation = [0, 1, 0]

    # Calculate inverse kinematics
    adjusted_angles_degrees, computed_position = calculate_ik(my_chain, target_position, target_orientation)
    
    # Calculate accuracy
    accuracy = calculate_accuracy(target_position, computed_position)
    
    if accuracy >= 95:
        # Create a DataFrame row with formatted data
        joint_data_df = pd.DataFrame([adjusted_angles_degrees + [x, y, z]], columns=[f'Joint {i+1}' for i in range(len(adjusted_angles_degrees))] + ['x', 'y', 'z'])
    
        # Extract joint angles for publishing
        joint_angles = joint_data_df.iloc[0, :-3].values.tolist()
    
        # Create a JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = rospy.Time.now()
        joint_state_msg.name = [f'joint_{i+1}' for i in range(len(joint_angles))]
        joint_state_msg.position = joint_angles
        
        # Publish the message
        joint_pub.publish(joint_state_msg)
        print(f"Accuracy is {accuracy:.2f}")
        print(joint_angles)
    else:
        print(f"Accuracy is {accuracy:.2f}")
        print("Accuracy is less than 95%. Joint states not published.")

def main():
    global my_chain, joint_pub
    # Initialize the ROS node
    rospy.init_node('joint_data_publisher', anonymous=True)
    
    # Create a publisher for joint states
    joint_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
    
    # Create a subscriber for ik_coordinates
    rospy.Subscriber('/ik_coordinates', Point, callback)
    
    # Load the robot chain from the URDF file
    #my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])
    #my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dofbot.urdf", active_links_mask=[True, True, True, True, True, False])
    #my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dofbot1.urdf", active_links_mask=[True, True, True, True, True, False])
    my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/100.urdf", active_links_mask=[False,True, True, True, True, True, False])

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass