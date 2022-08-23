#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.msg import ModelState


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
        self.initParams()
        rospy.Subscriber(f"{self.ns}command/cmd_vel", Twist, self.updateCmdVel)

        if self.real_controller:
            self.pose_publisher = rospy.Publisher(
                f"{self.ns}/command/pose", PoseStamped, queue_size=100)

        else:
            self.pose_publisher = rospy.Publisher(
                '/gazebo/set_model_state', ModelState, queue_size=100)

        self.publish_timer = rospy.Timer(rospy.Duration(self.tstep), self.publishGoalCallback)

    def initParams(self):
        self.tstep = rospy.get_param('vel_controller_tstep', 0.01)
        self.real_controller = rospy.get_param('vel_controller_real', False)
        self.ns = rospy.get_namespace()
        self.cmd_vel = Twist()
        self.current_pose = rospy.wait_for_message(f"{self.ns}ground_truth/pose", Pose)

    def publishGoalCallback(self, event):
        goal = self.getGoalPose()

        if self.real_controller:
            msg = PoseStamped()
            msg.pose = goal
            msg.header.stamp = rospy.Time.now()
            self.pose_publisher.publish(msg)
        else:
            msg = ModelState()

            msg.model_name = self.ns.replace('/', '')
            msg.pose = goal
            msg.twist = Twist()
            self.pose_publisher.publish(msg)
        self.current_pose = goal

    def getGoalPose(self):
        goal = Pose()

        goal.position.x = self.cmd_vel.linear.x * self.tstep + self.current_pose.position.x
        goal.position.y = self.cmd_vel.linear.y * self.tstep + self.current_pose.position.y
        goal.position.z = self.cmd_vel.linear.z * self.tstep + self.current_pose.position.z

        yaw = euler_from_quaternion(
            [self.current_pose.orientation.x,
             self.current_pose.orientation.y,
             self.current_pose.orientation.z,
             self.current_pose.orientation.w])[2]
        ori = quaternion_from_euler(
            0, 0,
            self.cmd_vel.angular.z * self.tstep + yaw)

        goal.orientation.x = ori[0]
        goal.orientation.y = ori[1]
        goal.orientation.z = ori[2]
        goal.orientation.w = ori[3]
        return goal

    def updateCmdVel(self, vel):
        self.cmd_vel = vel


if __name__ == "__main__":
    name_node = vel_controller_node()
    rospy.spin()
