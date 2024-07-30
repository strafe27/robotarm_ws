#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import JointState
from ikpy.chain import Chain
import numpy as np
import math
import pandas as pd

def main():
    # Initialize the ROS node
    rospy.init_node('fkik_publisher', anonymous=True)
    
    # Create a publisher for the 'joint_states' topic
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    # Load the robot chain from the URDF file
    my_chain = Chain.from_urdf_file("/home/dofbot/robotarm_ws/src/kinematics_pkg/urdf/dec.urdf", active_links_mask=[False, True, True, True, True, True, False])

    rate = rospy.Rate(0.2)
    
    while not rospy.is_shutdown():
        try:


            # Split the string by commas and convert each part to a float (or int if you prefer)
            target_position = [0.03, 0.15, 0.05]

            # Calculate inverse kinematics
            ik = my_chain.inverse_kinematics(target_position, orientation_mode="Y")

            # Convert radians to degrees and round to 2 decimal points
            joint_angles_degrees = [round(math.degrees(angle), 2) for angle in ik]
            
            rospy.loginfo(f"Calculated joint angles (degrees): {joint_angles_degrees}")

            # Create a DataFrame row with formatted data
            data_row = pd.DataFrame([joint_angles_degrees + [x, y, z]], columns=[f'Joint {i+1}' for i in range(len(ik))] + ['x', 'y', 'z'])

            # Print the DataFrame
            rospy.loginfo("Joint Angles DataFrame:")
            rospy.loginfo(data_row)

            # Publish the joint state message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = rospy.Time.now()
            joint_state_msg.name = [f'joint_{i+1}' for i in range(len(ik))]
            joint_state_msg.position = ik.tolist()
            
            pub.publish(joint_state_msg)

            rate.sleep()

        except ValueError as e:
            rospy.logerr(f"ValueError: {e}")
        except Exception as e:
            rospy.logerr(f"An error occurred: {e}")

if __name__ == '__main__':
    main()
