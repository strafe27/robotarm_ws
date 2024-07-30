#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

def main():
    rospy.init_node('coordinate_publisher')

    ik_pub = rospy.Publisher('ik_coordinates', Point, queue_size=10)
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    
    while not rospy.is_shutdown():

        coordinates = input("Guide \nIK: y > 0.1\nFK: Extra careful! \nEnter 3 values for IK or 6 values for FK: ")

        # Split the string by commas and convert each part to a float
        target_position = [float(coord) for coord in coordinates.split(',')]

        if len(target_position) == 1:
            joint_state = JointState()
            joint_state.position = target_position
            joint_pub.publish(joint_state)
            rospy.loginfo(f"FK mode")
            rospy.loginfo(f"Published Joint States:\n{joint_state.position}")
    
        
        if len(target_position) == 3:
            point = Point()
            point.x, point.y, point.z = target_position
            ik_pub.publish(point)
            rospy.loginfo(f"IK mode")
            rospy.loginfo(f"Published IK coordinates:\n{point}")

        if len(target_position) == 5:
            joint_state = JointState()
            joint_state.position = target_position
            joint_pub.publish(joint_state)
            rospy.loginfo(f"FK mode")
            rospy.loginfo(f"Published Joint States:\n{joint_state.position}")

        if len(target_position) == 6:
            joint_state = JointState()
            joint_state.position = target_position
            joint_pub.publish(joint_state)
            rospy.loginfo(f"FK mode")
            rospy.loginfo(f"Published Joint States:\n{joint_state.position}")

        else:
            rospy.logwarn("Invalid input length. Please enter either 1,3,5 or 6 coordinates.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass