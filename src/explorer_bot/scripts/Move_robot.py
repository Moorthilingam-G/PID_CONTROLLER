#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_robot():
    rospy.init_node('move_robot', anonymous=True)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    twist = Twist()
    twist.linear.x = 0.5  # Forward velocity
    twist.angular.z = 0.0  # No angular velocity

    while not rospy.is_shutdown():
        cmd_vel_pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_robot()
    except rospy.ROSInterruptException:
        pass

