#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
import math

class ObstacleCondition:
    def __init__(self):
        # ===== THRESHOLD JARAK =====
        self.danger_dist  = 0.45   # lebih JAUH dari sebelumnya
        self.warning_dist = 0.70

        # ===== MIN DIST PER ZONA =====
        self.front_min = float('inf')
        self.left_min  = float('inf')
        self.right_min = float('inf')

        rospy.Subscriber("/scan", LaserScan, self.scan_callback)

    def scan_callback(self, msg):
        front, left, right = [], [], []

        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue

            angle = msg.angle_min + i * msg.angle_increment

            # ===== DEFINISI ZONA SUDUT =====
            if -math.radians(30) <= angle <= math.radians(30):
                front.append(r)

            elif math.radians(30) < angle <= math.radians(90):
                left.append(r)

            elif -math.radians(90) <= angle < -math.radians(30):
                right.append(r)

        self.front_min = min(front) if front else float('inf')
        self.left_min  = min(left)  if left  else float('inf')
        self.right_min = min(right) if right else float('inf')

    # ===== CONDITION =====
    def is_danger(self):
        return self.front_min < self.danger_dist

    def is_warning(self):
        return self.front_min < self.warning_dist

    # ===== ARAH AMAN =====
    def safer_direction(self):
        """
        return:
        +1  → belok kiri
        -1  → belok kanan
        """
        if self.left_min > self.right_min:
            return +1
        else:
            return -1

