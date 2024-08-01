#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, Int32
import ikpy.chain
import ikpy.utils.plot as plot_utils
import numpy as np
import time
import math

my_chain = ikpy.chain.Chain.from_urdf_file("dec.urdf", active_links_mask=[False, True, True, True, True, True, False])

# Declare global variables
coordinates = None

def calculate_ik(target_position, target_orientation):
    # Perform inverse kinematics to find joint angles
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Y")
    adjusted_angles = [angle + math.radians(90) for angle in ik]
    adjusted_angles_degrees = list(map(lambda r: round(math.degrees(r), 2), adjusted_angles))
    print("The angles of each joint, adjusted by adding 90 degrees, are:", adjusted_angles_degrees)
    computed_position = my_chain.forward_kinematics(ik)
    print("Computed position (readable): %s" % ['%.2f' % elem for elem in computed_position[:3, 3]])
    return adjusted_angles_degrees, computed_position[:3, 3]

def calculate_accuracy(input_position, computed_position):
    error = np.linalg.norm(np.array(input_position) - np.array(computed_position))
    accuracy = 100 * (1 - error / np.linalg.norm(np.array(input_position)))
    return accuracy

def callback_real_world_coordinates(data):
    global coordinates, x, y, z
    x = 0
    y = 0
    z = 0
    if x == 0 or y == 0 or z == 0:
        # Extract x, y, and z from received data
        x, y, z = data.data[:3]
        # Assume z coordinate is 0.04 (as in the original code)
        if z == 1111:
            print("Blue")
        elif z == 2222:
            print("Green")
        elif z == 3333:
            print("Unknown")
        print("X location:", x)
        print("Y location:", y)
        coordinates.unregister()

def pause():
    user_input = input("Type 'C' to continue: ")
    return user_input.lower() == 'c'

def publisher_node():
    global x, y, coordinates
    rospy.init_node('picknplace', anonymous=True)
    angle_pub = rospy.Publisher('servo_angles', Float64MultiArray, queue_size=10)
    gripper_pub = rospy.Publisher('servo6_angle', Int32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        servo_angle = 50
        angle_pub.publish(Float64MultiArray(data=[servo_angle]))
        rospy.loginfo("Open")
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        servo_angles = [90, 90, 0, 0, 90]
        rospy.loginfo_throttle(1, servo_angles)
        angle_pub.publish(Float64MultiArray(data=servo_angles))
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        print("Scanning in progress")
        coordinates = rospy.Subscriber('real_world_coordinates', Float64MultiArray, callback_real_world_coordinates)
        time.sleep(2)
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        target_position = [x, y, 0.00]
        target_orientation = [0, -1, 0]
        # Calculate and print inverse kinematics
        adjusted_angles_degrees, computed_position = calculate_ik(target_position, target_orientation)
        accuracy = calculate_accuracy(target_position, computed_position)
        # Check if accuracy is less than 95% before publishing
        if accuracy >= 99:
            # Select only angles 2, 3, 4, 5, and 6
            selected_angles = adjusted_angles_degrees[1:6]
            # Publish selected angles
            adjusted_angles_msg = Float64MultiArray()
            adjusted_angles_msg.data = selected_angles
            angle_pub.publish(adjusted_angles_msg)
            print("Published angles.")
        else:
            print("Accuracy is less than 95%. Angles not published.")
            print("Accuracy: %.2f%%" % accuracy)
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        servo_angle = 165
        angle_pub.publish(Float64MultiArray(data=[servo_angle]))
        rospy.loginfo("Gripper close")
        if not pause():
            rospy.loginfo("Invalid input. Exiting.")
            break
        target_position = [x, y, 0.02]
        target_orientation = [0, -1, 0]
        # Calculate and print inverse kinematics
        adjusted_angles_degrees, computed_position = calculate_ik(target_position, target_orientation)
        accuracy = calculate_accuracy(target_position, computed_position)
        # Check if accuracy is less than 95% before publishing
        if accuracy >= 99:
            # Select only angles 2, 3, 4, 5, and 6
            selected_angles = adjusted_angles_degrees[1:6]
            # Publish selected angles
            adjusted_angles_msg = Float64MultiArray()
            adjusted_angles_msg.data = selected_angles
            angle_pub.publish(adjusted_angles_msg)
            print("Published angles.")
        else:
            print("Accuracy is less than 95%. Angles not published.")
            print("Accuracy: %.2f%%" % accuracy)
        if z == 1111:
            print("Going to red collection point")
            servo_angles = [0, 48, 12, 31, 89]
        elif z == 2222:
            print("Going to green collection point")
            servo_angles = [180, 48, 12, 31, 89]
        elif z == 3333:
            print("Colour is unknown")
            servo_angles = [180, 48, 12, 31, 89]
        rospy.loginfo_throttle(1, servo_angles)
        angle_pub.publish(Float64MultiArray(data=servo_angles))

if __name__ == '__main__':
    try:
        publisher_node()
    except rospy.ROSInterruptException:
        pass
