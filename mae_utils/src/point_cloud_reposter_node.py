#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud2


class point_cloud_reposter_node:
    """ 
    This node subscribes to the lidar sensors of all the uavs and 
    republishes the point clouds to a single topic so that a global
    map can be made. 
    """

    def __init__(self):
        rospy.init_node("point_cloud_reposter_node")
        rospy.loginfo("Starting point_cloud_reposter_Node")
        rospy.Subscriber('/drone1/velodyne_points', PointCloud2,
                         self.post_pcl)
        rospy.Subscriber('/drone2/velodyne_points', PointCloud2,
                         self.post_pcl)
        rospy.Subscriber('/drone3/velodyne_points', PointCloud2,
                         self.post_pcl)
        self.pcl_publisher = rospy.Publisher(
            '/combined_pcl', PointCloud2, queue_size=100)

    def post_pcl(self, pcl):
        self.pcl_publisher.publish(pcl)


if __name__ == "__main__":
    name_node = point_cloud_reposter_node()
    rospy.spin()
