#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class MoveForwardAction:
    def __init__(self, obstacle):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.obstacle = obstacle

    def run(self):
        twist = Twist()

        # ===== SPEED ADAPTIVE =====
        if self.obstacle.is_warning():
            twist.linear.x = 0.1   # pelan
        else:
            twist.linear.x = 0.34   # normal

        twist.angular.z = 0.0
        self.cmd_pub.publish(twist)

        rospy.loginfo(
            "BT: Moving forward | front=%.2f left=%.2f right=%.2f",
            self.obstacle.front_min,
            self.obstacle.left_min,
            self.obstacle.right_min
        )

        return "SUCCESS"

