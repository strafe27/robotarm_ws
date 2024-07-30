#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point
from time import sleep

def main():
    rospy.init_node('coordinate_publisher')

    ik_pub = rospy.Publisher('ik_coordinates', Point, queue_size=10)

    # Define a list of coordinates to publish
    coordinates_list = [
        (0.00, 0.13, 0.03),
        (0.02, 0.11, 0.03),
        (0.02, 0.12, 0.03),
        (0.02, 0.13, 0.03),
        (0.02, 0.14, 0.03),
        (0.02, 0.15, 0.03),
        (0.01, 0.15, 0.03),
        (0.01, 0.14, 0.03),
        (0.01, 0.13, 0.03),
        (0.01, 0.12, 0.03),
        (0.01, 0.11, 0.03),
        (0.0, 0.11, 0.03),
        (0.0, 0.12, 0.03),
        (0.0, 0.13, 0.03),
        (0.0, 0.14, 0.03),
        (0.0, 0.15, 0.03),
        (-0.01, 0.15, 0.03),
        (-0.01, 0.14, 0.03),
        (-0.01, 0.13, 0.03),
        (-0.01, 0.12, 0.03),
        (-0.01, 0.11, 0.03),
        (-0.02, 0.11, 0.03),
        (-0.02, 0.12, 0.03),
        (-0.02, 0.13, 0.03),
        (-0.02, 0.14, 0.03),
        (-0.02, 0.15, 0.03)
    ]

    rate = rospy.Rate(1)  # Publish rate of 1 Hz (one message per second)

    for coordinates in coordinates_list:
        if rospy.is_shutdown():
            break

        point = Point()
        point.x, point.y, point.z = coordinates
        ik_pub.publish(point)
        rospy.loginfo(f"Published IK coordinates:\n{point}")

        sleep(3)  # Sleep for 3 seconds

    rospy.loginfo("Finished publishing all IK coordinates")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass