#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class StopAction:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def run(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        self.cmd_pub.publish(twist)
        rospy.loginfo("BT: STOP – Coverage completed")

        return "SUCCESS"

