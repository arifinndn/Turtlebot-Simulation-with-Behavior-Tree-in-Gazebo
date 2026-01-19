#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class RotateAction:
    def __init__(self, obstacle):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.obstacle = obstacle
        self.rate = rospy.Rate(10)

    def run(self):
        rospy.loginfo("BT: Rotating away from obstacle")

        twist = Twist()
        twist.linear.x = 0.0

        direction = self.obstacle.safer_direction()
        twist.angular.z = 1 * direction

        while not rospy.is_shutdown():
            if not self.obstacle.is_danger():
                break
            self.cmd_pub.publish(twist)
            self.rate.sleep()

        self.cmd_pub.publish(Twist())
        rospy.sleep(0.2)

        return "SUCCESS"

