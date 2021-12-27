#!/usr/bin/env python


import rospy
#from geometry_msgs.msg import PoseStamped


def callback(data):
    rospy.loginfo("done")


def listener():

    rospy.init_node('listener', anonymous=True)

   # ss = rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
