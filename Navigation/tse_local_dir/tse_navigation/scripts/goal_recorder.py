#!/usr/bin/env python
import rospy
import json
from geometry_msgs.msg import PoseStamped

class GoalRecorder:
    def __init__(self):
        self.goals = []
        self.output_file = '/tmp/static_world_goals.json'
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.callback)

    def callback(self, msg):
        goal = {
            'frame_id': msg.header.frame_id,
            'position': {'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z},
            'orientation': {'x': msg.pose.orientation.x, 'y': msg.pose.orientation.y, 'z': msg.pose.orientation.z, 'w': msg.pose.orientation.w}
        }
        self.goals.append(goal)
        
        with open(self.output_file, 'w') as f:
            json.dump(self.goals, f, indent=2)
        rospy.loginfo(f"Recorded goal {len(self.goals)}")

if __name__ == '__main__':
    rospy.init_node('goal_recorder')
    GoalRecorder()
    rospy.spin()
