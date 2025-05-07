#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry

class OdomCalculator:
    def __init__(self):
        self.distance = 0.0
        self.last_x = None
        self.last_y = None
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.odom_cb)
        rospy.loginfo("Odometry distance calculator ready")

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_x is not None:
            self.distance += ((x-self.last_x)**2 + (y-self.last_y)**2)**0.5
        self.last_x, self.last_y = x, y
        rospy.loginfo_throttle(1, f"Traveled distance: {self.distance:.2f}m")

if __name__ == "__main__":
    rospy.init_node("distance_calculator")
    OdomCalculator()
    rospy.spin()
