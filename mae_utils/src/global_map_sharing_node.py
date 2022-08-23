#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Point, PointStamped
from functools import partial
import sensor_msgs.point_cloud2 as pc2
from octomap_msgs.srv import BoundingBoxQuery, BoundingBoxQueryRequest
from nav_msgs.msg import OccupancyGrid


class global_map_sharing_node:
    """
    This node subscribes to the lidar sensors of all the uavs and
    republishes the point clouds to a single topic so that a global
    map can be made.
    """

    def __init__(self):
        rospy.init_node("global_map_sharing_node")

        self.drones = dict()
        self.pcl_publisher = rospy.Publisher(
            '/combined_pcl', PointCloud2, queue_size=100)
        self.map_publisher = rospy.Publisher('/filtered_map', OccupancyGrid, queue_size=100)
        rospy.Subscriber('/projected_map', OccupancyGrid, self.filterMap)
        rospy.Timer(rospy.Duration(0.5), self.checkForDrones)

    def checkForDrones(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find("drone")
            if str_loc != -1:
                drone_id = int(topic[0][str_loc + len("drone")])
                if drone_id not in self.drones.keys():
                    self.drones[drone_id] = Point()
                    rospy.Subscriber(
                        f"/drone{drone_id}/ground_truth/position", PointStamped, partial(self.updateLocations, drone_id))

                    rospy.Subscriber(f"/drone{drone_id}/velodyne_points", PointCloud2,
                                     self.postPCL, drone_id)

    def postPCL(self, pcl, drone_id):
        pcl_generator = pc2.read_points(pcl)

        self.pcl_publisher.publish(pcl)

    def updateLocations(self, drone_id, msg):
        self.drones[drone_id] = msg.point

    def dist(self, point1, point2):
        return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

    def filterMap(self, map_msg):
        map_msg.data = list(map_msg.data)

        for point in self.drones.values():
            # point to 2dmap index
            grid_x = int((point.x - map_msg.info.origin.position.x) / map_msg.info.resolution)
            grid_y = int((point.y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            for i in range(grid_x - 2, grid_x + 3):
                for j in range(grid_y - 2, grid_y + 3):
                    map_msg.data[int(i + j * map_msg.info.width)] = 0

        self.map_publisher.publish(map_msg)


if __name__ == "__main__":
    name_node = global_map_sharing_node()
    rospy.spin()
