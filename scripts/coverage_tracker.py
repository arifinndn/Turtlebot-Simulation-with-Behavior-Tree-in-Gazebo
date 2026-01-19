#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from std_msgs.msg import Float32
from geometry_msgs.msg import Point

class CoverageTracker:
    def __init__(self):
        rospy.init_node("coverage_tracker")

        # ===== PARAMETER LAPANGAN =====
        self.map_size = 6.0      # meter (6x6)
        self.resolution = 0.4    # meter per cell

        self.grid_size = int(self.map_size / self.resolution)
        self.total_cells = self.grid_size * self.grid_size

        self.grid = [[0 for _ in range(self.grid_size)]
                     for _ in range(self.grid_size)]

        self.cleaned_cells = 0

        # ===== ORIGIN MAP =====
        self.origin_x = -self.map_size / 2.0
        self.origin_y = -self.map_size / 2.0

        # ===== ROS =====
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        self.marker_pub = rospy.Publisher(
            "/coverage_marker", Marker, queue_size=10
        )

        self.coverage_pub = rospy.Publisher(
            "/coverage_percent", Float32, queue_size=10
        )

        self.init_marker()

    def init_marker(self):
        self.marker = Marker()
        self.marker.header.frame_id = "odom"
        self.marker.ns = "coverage_trail"
        self.marker.id = 0
        self.marker.type = Marker.POINTS
        self.marker.action = Marker.ADD

        self.marker.scale.x = 0.05
        self.marker.scale.y = 0.05

        self.marker.color.r = 0.0
        self.marker.color.g = 0.0
        self.marker.color.b = 1.0
        self.marker.color.a = 1.0

    def world_to_grid(self, x, y):
        gx = int((x - self.origin_x) / self.resolution)
        gy = int((y - self.origin_y) / self.resolution)
        return gx, gy

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        gx, gy = self.world_to_grid(x, y)

        if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
            if self.grid[gx][gy] == 0:
                self.grid[gx][gy] = 1
                self.cleaned_cells += 1

                coverage = self.cleaned_cells / self.total_cells
                self.coverage_pub.publish(coverage)

                rospy.loginfo(
                    "Coverage: %.2f %%",
                    coverage * 100.0
                )

                self.add_marker_point(x, y)

    def add_marker_point(self, x, y):
        p = Point()
        p.x = x
        p.y = y
        p.z = 0.01

        self.marker.points.append(p)
        self.marker.header.stamp = rospy.Time.now()
        self.marker_pub.publish(self.marker)


if __name__ == "__main__":
    CoverageTracker()
    rospy.spin()

