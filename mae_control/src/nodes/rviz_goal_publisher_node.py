#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped

class rviz_goal_publisher_node:
    def __init__(self):
        rospy.init_node("rviz_goal_publisher_node")
        rospy.loginfo("Starting waypoint_node as name_node.")
        ns = rospy.get_namespace()
        self.lee_publisher = rospy.Publisher(f'/{ns}/command/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.callback)

    def callback(self, goal_pose):
        goal_pose.pose.position.z = rospy.get_param('~altitude', default=1)
        self.lee_publisher.publish(goal_pose)

if __name__ == "__main__":
    name_node = rviz_goal_publisher_node()
    rospy.spin()
