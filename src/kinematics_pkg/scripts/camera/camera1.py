#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
import numpy as np
import time

# Define callback for real world coordinates
def callback_real_world_coordinates(data):
    global x, y, z
    x, y, z = data.data

# Pause function that continues when 'c' is pressed
def pause():
    user_input = input("Press 'c' to continue: ")
    return user_input.lower() == 'c'

def service_node():
    global x, y, z, coordinates

    x = y = z = 0.0
    coordinates = None

    rospy.init_node('picknplace', anonymous=True)
    ik_pub = rospy.Publisher('ik_coordinates', Point, queue_size=10)
    joint_pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        servo_angle = 50
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = [servo_angle]
        joint_pub.publish(joint_state)
        rospy.loginfo("Open")

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        servo_angles = [90, 90, 0, 0, 90]
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = servo_angles
        rospy.loginfo_throttle(1, servo_angles)
        joint_pub.publish(joint_state)

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        print("Scanning in progress")
        rospy.Subscriber('real_world_coordinates', Float64MultiArray, callback_real_world_coordinates)
        time.sleep(1)

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        target_position = [x, y, 0.0]
        print(target_position)
        point = Point(x=target_position[0], y=target_position[1], z=target_position[2])
        ik_pub.publish(point)

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        servo_angle = 155
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = [servo_angle]
        joint_pub.publish(joint_state)
        rospy.loginfo("Gripper close")

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        target_position = [x, y, 0.02]
        print(target_position)
        point = Point(x=target_position[0], y=target_position[1], z=target_position[2])
        ik_pub.publish(point)

        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break

        if z == 1111:
            print("Going to green collection point")
            servo_angles = [0, 45, 15, 28, 90]
        elif z == 2222:
            print("Going to blue collection point")
            servo_angles = [180, 45, 15, 28, 90]
        elif z == 3333:
            print("Colour is unknown")
            servo_angles = [180, 48, 12, 31, 89]

        joint_state = JointState()
        joint_state.header.stamp = rospy.Time.now()
        joint_state.position = servo_angles
        rospy.loginfo_throttle(1, servo_angles)
        joint_pub.publish(joint_state)

if __name__ == '__main__':
    try:
        service_node()
    except rospy.ROSInterruptException:
        pass
