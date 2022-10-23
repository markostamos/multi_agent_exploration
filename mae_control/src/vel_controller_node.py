#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from gazebo_msgs.msg import ModelState


class vel_controller_node:
    """
    Simple forward integration velocity controller based on an implemented position controller
    new_pose = velocity * timestep + old_pose

    For simulation purposes if no real position controller is provided,
    the position controller is simulated by publishing the new pose to the gazebo model state topic.
    """

    def __init__(self):

        rospy.init_node("vel_controller_node")
        self.initParams()
        rospy.Subscriber(f"{self.ns}command/cmd_vel", Twist, self.updateCmdVel)
        rospy.Subscriber(f"{self.ns}command/position", Point, self.updatePosition)

        self.goal_position = Point()
        self.position_control = False
        if self.real_controller:
            self.pose_publisher = rospy.Publisher(
                f"/pose_out", PoseStamped, queue_size=100)

        else:
            self.pose_publisher = rospy.Publisher(
                '/gazebo/set_model_state', ModelState, queue_size=100)

        self.publish_timer = rospy.Timer(rospy.Duration(self.tstep), self.publishGoalCallback)

    def initParams(self):
        self.tstep = rospy.get_param('~tstep', 0.01)
        self.real_controller = rospy.get_param('~real_controller', False)
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

        if self.position_control:
            goal = Pose()
            goal.position = self.goal_position
            goal.orientation = self.current_pose.orientation
            self.position_control = False
            return goal
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
        if vel.linear.x != vel.linear.x:
            vel.linear.x = 0
        if vel.linear.y != vel.linear.y:
            vel.linear.y = 0
        if vel.linear.z != vel.linear.z:
            vel.linear.z = 0
        self.cmd_vel = vel

    def updatePosition(self, pos):
        self.goal_position = pos
        self.position_control = True


if __name__ == "__main__":
    name_node = vel_controller_node()
    rospy.spin()
