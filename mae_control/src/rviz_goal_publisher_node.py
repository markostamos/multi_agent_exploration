#!/usr/bin/env python


import rospy
from geometry_msgs.msg import PoseStamped, Quaternion


class rviz_goal_publisher_node:
    def __init__(self):
        rospy.init_node("rviz_goal_publisher_node")
        rospy.loginfo("Starting waypoint_node as name_node.")
        ns = rospy.get_namespace()
        self.lee_publisher = rospy.Publisher(
            f'/{ns}/command/pose', PoseStamped, queue_size=10)
        rospy.Subscriber(f'/{ns}/move_base_simple/goal',
                         PoseStamped, self.callback)

    def callback(self, goal_pose):

        goal_pose.pose.position.z = 1

        goal_pose.pose.orientation = Quaternion()
        self.lee_publisher.publish(goal_pose)


if __name__ == "__main__":
    name_node = rviz_goal_publisher_node()
    rospy.spin()
