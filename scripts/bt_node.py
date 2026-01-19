#!/usr/bin/env python3

import rospy
from obstacle_condition import ObstacleCondition
from move_forward_action import MoveForwardAction
from rotate_action import RotateAction
from stop_action import StopAction
from coverage_done_condition import CoverageDoneCondition

class BehaviorTree:
    def __init__(self):
        rospy.init_node("bt_turtlebot_node")

        # ===== CONDITIONS =====
        self.obstacle = ObstacleCondition()
        self.coverage_done = CoverageDoneCondition()

        # ===== ACTIONS =====
        self.stop = StopAction()
        self.rotate = RotateAction(self.obstacle)
        self.move = MoveForwardAction(self.obstacle)

        self.rate = rospy.Rate(10)

    def tick(self):
        # ===== PRIORITY SELECTOR =====
        if self.coverage_done.check():
            self.stop.run()

        elif self.obstacle.is_danger():
            self.stop.run()
            self.rotate.run()

        else:
            self.move.run()

    def run(self):
        while not rospy.is_shutdown():
            self.tick()
            self.rate.sleep()

if __name__ == "__main__":
    BehaviorTree().run()

