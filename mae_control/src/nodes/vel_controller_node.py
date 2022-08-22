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

        self.tstep = rospy.get_param('vel_controller_tstep', 0.01)
        self.real_controller = rospy.get_param('vel_controller_real', False)
        self.curr_pose = Pose()
        self.curr_vel = Twist()
        self.ns = rospy.get_namespace()

        self.curr_pose = rospy.wait_for_message(
            f'{self.ns}ground_truth/pose', Pose)

        rospy.Subscriber(f'{self.ns}command/cmd_vel', Twist, self.update_vel)
        rospy.Subscriber(f'{self.ns}command/pose', PoseStamped, self.update_pose)
        self.lee_publisher = rospy.Publisher(f'/{self.ns}/command/pose',
                                             PoseStamped, queue_size=10)
        self.gazebo_publisher = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

        if self.real_controller:
            self.lee_pub_timer = rospy.Timer(rospy.Duration(self.tstep), self.lee_pub_callback)
        else:
            self.gazebo_pub_timer = rospy.Timer(
                rospy.Duration(self.tstep), self.gazebo_pub_callback)

    def lee_pub_callback(self, n):
        print(self.ns)

        if self.curr_vel == Twist():
            return

        pose = self.curr_pose

        yaw = euler_from_quaternion(
            [pose.orientation.x,
             pose.orientation.y,
             pose.orientation.z,
             pose.orientation.w])[2]

        # Goal Pose
        goal = PoseStamped()

        goal.header.stamp = rospy.Time.now()

        goal.pose.position.x = self.curr_vel.linear.x * self.tstep + pose.position.x
        goal.pose.position.y = self.curr_vel.linear.y * self.tstep + pose.position.y
        #goal.pose.position.z = self.curr_vel.linear.z * self.tstep + pose.position.z
        goal.pose.position.z = 0
        if goal.pose.position.z < 0:
            goal.pose.position.z = 0

        ori = quaternion_from_euler(
            0, 0,
            self.curr_vel.angular.z * self.tstep + yaw)

        goal.pose.orientation.x = ori[0]
        goal.pose.orientation.y = ori[1]
        goal.pose.orientation.z = ori[2]
        goal.pose.orientation.w = ori[3]

        self.lee_publisher.publish(goal)

    def gazebo_pub_callback(self, n):

        goal = Pose()
        goal.position.x = self.curr_vel.linear.x * self.tstep
        goal.position.y = self.curr_vel.linear.y * self.tstep
        goal.position.z = self.curr_vel.linear.z * self.tstep

        if self.curr_pose.position.z > 1:
            goal.position.z = -0.3 * self.tstep

        orientation = quaternion_from_euler(0, 0, 0)

        goal.orientation.x = orientation[0]
        goal.orientation.y = orientation[1]
        goal.orientation.z = orientation[2]
        goal.orientation.w = orientation[3]

        model_state = ModelState()

        model_state.model_name = self.ns.replace('/', '')
        model_state.pose = goal
        model_state.twist = Twist()
        model_state.reference_frame = self.ns.replace('/', '')

        self.gazebo_publisher.publish(model_state)

    def update_pose(self, msg):
        self.curr_pose = msg.pose

    def update_vel(self, vel):
        self.curr_vel = vel


if __name__ == "__main__":
    name_node = vel_controller_node()
    rospy.spin()
