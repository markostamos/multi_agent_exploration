#!/usr/bin/env python
import rospy

if __name__ == "__main__":
    rospy.init_node("name_node")
    rospy.loginfo("Starting name_node.")

    while not rospy.is_shutdown():
        pass
