#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time
import random

class TurnAction:
    def __init__(self):
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def run(self):
        twist = Twist()

        # ===============================
        # 1. ROTASI LEBIH BESAR
        # ===============================
        twist.linear.x = 0.0

        # random kiri / kanan
        direction = random.choice([-1, 1])
        twist.angular.z = direction * 1   # lebih agresif

        rotate_time = random.uniform(1.8, 2.4)  # ±100–140 derajat

        start_time = time.time()
        while time.time() - start_time < rotate_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

        # ===============================
        # 2. MAJU CLEARANCE LEBIH JAUH
        # ===============================
        twist.angular.z = 0.0
        twist.linear.x = 0.20

        forward_time = 1.2  # lebih jauh dari sebelumnya

        start_time = time.time()
        while time.time() - start_time < forward_time and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rospy.sleep(0.1)

        rospy.loginfo("BT: Strong turn + long clear")
        return "SUCCESS"

