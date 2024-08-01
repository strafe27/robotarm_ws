#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Int32
import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import time
import math

def service_node():
    global x, y, coordinates

    rospy.init_node('picknplace', anonymous=True)
    ik_pub = rospy.Publisher('ik_coordinates', Point, queue_size=10)
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        servo_angle = 50
        joint_pub.publish(Float64MultiArray(data=[servo_angle]))
        rospy.loginfo("Open")
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        servo_angles = [90, 90, 0, 0, 90]
        rospy.loginfo_throttle(1, servo_angles)
        joint_pub.publish(Float64MultiArray(data=servo_angles))
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        print("Scanning in progress")
        coordinates = rospy.Subscriber('real_world_coordinates', Float64MultiArray, callback_real_world_coordinates)
        time.sleep(2)
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        target_position = [x, y, 0.01]
        point = Point(x=target_position[0], y=target_position[1], z=target_position[2])
        ik_pub.publish(point)

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        servo_angle = 165
        joint_pub.publish(Float64MultiArray(data=[servo_angle]))
        rospy.loginfo("Gripper close")

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        
        if z == 1111:
            print("Going to red collection point")
            servo_angles = [0, 45, 15, 28, 90]
        elif z == 2222:
            print("Going to green collection point")
            servo_angles = [180, 45, 15, 28, 90]
        elif z == 3333:
            print("Colour is unknown")
            servo_angles = [180, 48, 12, 31, 89]
        rospy.loginfo_throttle(1, servo_angles)
        joint_pub.publish(Float64MultiArray(data=servo_angles))

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        pass