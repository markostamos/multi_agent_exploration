#!/usr/bin/env python
import rospy
import tf
from tf.msg import tfMessage
from geometry_msgs.msg import TransformStamped, Pose


class base_footprint_publisher_node:
    def __init__(self):
        rospy.init_node("base_footprint_publisher_node")
        rospy.loginfo("Starting base_footprint_publisher_node as name_node.")
        self.pub_tf = rospy.Publisher("/tf", tfMessage, queue_size=10)
        self.height = 0
        self.mav_name = rospy.get_namespace()

        rospy.Timer(rospy.Duration(1.0 / 10), self.publish_transform)
        rospy.Subscriber(f'/{self.mav_name}/ground_truth/pose', Pose,
                         self.update_height)

    def publish_transform(self, tmp):

        t = TransformStamped()
        t.header.frame_id = f'{self.mav_name}base_link'
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = f'{self.mav_name}base_footprint'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = -self.height

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        tfm = tfMessage([t])
        self.pub_tf.publish(tfm)

    def update_height(self, pose):
        self.height = pose.position.z


if __name__ == "__main__":
    name_node = base_footprint_publisher_node()
    rospy.spin()
