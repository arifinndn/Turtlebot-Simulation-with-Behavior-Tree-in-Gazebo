#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

class CoverageDoneCondition:
    def __init__(self):
        self.coverage = 0.0
        self.threshold = 0.2  # 20%

        rospy.Subscriber(
            "/coverage_percent",
            Float32,
            self.callback
        )

    def callback(self, msg):
        self.coverage = msg.data

    def check(self):
        if self.coverage >= self.threshold:
            rospy.loginfo(
                "BT: Coverage DONE (%.2f %%)",
                self.coverage * 100.0
            )
            return True
        return False

