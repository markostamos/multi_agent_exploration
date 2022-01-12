#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion


class vel_controller_node:
    """
    Simple forward integration velocity controller based on the lee position controller
    Subscribes to /<drone_name>/command/cmd_vel topic
    Publishes a desired pose to /<drone_name>/command/pose every tstep seconds via the lee_pub_callback function

    new_pose = velocity * timestep + old_pose

    When cmd_vel is 0 nothing is published to allow the use of the position controller when this one is not used.
    """

    def __init__(self):

        rospy.init_node("vel_controller_node")

        self.tstep = rospy.get_param('vel_controller_tstep', 0.1)
        self.curr_pose = PoseStamped()
        self.curr_vel = Twist()
        ns = rospy.get_namespace()

        self.curr_pose.pose = rospy.wait_for_message(
            f'/{ns}/ground_truth/pose', Pose)

        rospy.Subscriber(f'/{ns}/command/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber(f'/{ns}/command/pose', PoseStamped, self.update_pose)
        self.lee_publisher = rospy.Publisher(f'/{ns}/command/pose',
                                             PoseStamped, queue_size=10)

        lee_pub_timer = rospy.Timer(rospy.Duration(
            self.tstep), self.lee_pub_callback)

    def check_zero_vel(self, vel):
        velx = vel.linear.x
        vely = vel.linear.y
        velz = vel.linear.z
        velyaw = vel.angular.z

        if velx == 0 and vely == 0 and velz == 0 and velyaw == 0:
            return True
        return False

    def lee_pub_callback(self, n):

        # if cmd_vel = 0  don't do anything
        if self.check_zero_vel(self.curr_vel):
            return

        velx = self.curr_vel.linear.x
        vely = self.curr_vel.linear.y
        velz = self.curr_vel.linear.z
        velyaw = self.curr_vel.angular.z

        posx = self.curr_pose.pose.position.x
        posy = self.curr_pose.pose.position.y
        posz = self.curr_pose.pose.position.z
        orx = self.curr_pose.pose.orientation.x
        ory = self.curr_pose.pose.orientation.y
        orz = self.curr_pose.pose.orientation.z
        orw = self.curr_pose.pose.orientation.w

        yaw = euler_from_quaternion([orx, ory, orz, orw])[2]

        goal = PoseStamped()

        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x = velx * self.tstep + posx
        goal.pose.position.y = vely * self.tstep + posy
        goal.pose.position.z = velz * self.tstep + posz
        if goal.pose.position.z < 0:
            goal.pose.position.z = 0
        ori = quaternion_from_euler(0, 0, velyaw * self.tstep + yaw)

        goal.pose.orientation.x = ori[0]
        goal.pose.orientation.y = ori[1]
        goal.pose.orientation.z = ori[2]
        goal.pose.orientation.w = ori[3]

        self.lee_publisher.publish(goal)

    def update_pose(self, pose):
        self.curr_pose = pose

    def update_vel(self, vel):
        self.curr_vel = vel


if __name__ == "__main__":
    name_node = vel_controller_node()
    rospy.spin()
