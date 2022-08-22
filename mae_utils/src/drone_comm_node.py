#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
from functools import partial
from math import sqrt
import sensor_msgs.point_cloud2 as pc2


class DroneCommNode:
    """
        Simulates drone communication.
        Each drone runs an instance of this node.
    """

    def __init__(self):
        rospy.init_node("drone_comm_node")
        rospy.loginfo("Starting drone_comm_node")

        self.Initialize()
        self.CreatePublishers()
        self.CreateTimers()

    def Initialize(self):
        self.drone_id = 1 if rospy.get_namespace() == "/" else int(rospy.get_namespace()[-2])
        self.map_sharing_freq = rospy.get_param("~map_sharing_freq", 0.5)
        self.loc_sharing_freq = rospy.get_param("~loc_sharing_freq", 10)
        self.plan_sharing_freq = rospy.get_param("~plan_sharing_freq", 1)
        self.max_comm_dist = rospy.get_param("~max_comm_dist", 200)

        self.drones = []

        self.drone_locs = dict()
        self.drone_maps = dict()
        self.myloc = Point()

    def CheckForDrones(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find("drone")
            if str_loc != -1:
                drone_id = int(topic[0][str_loc + len("drone")])
                if drone_id not in self.drones:
                    self.drones.append(drone_id)
                    rospy.Subscriber(
                        f"/drone{drone_id}/ground_truth/position", PointStamped, partial(self.UpdateLocations, drone_id))

                    rospy.Subscriber(f"/drone{drone_id}/octomap_point_cloud_centers",
                                     PointCloud2, partial(self.UpdateMaps, drone_id))

    def CreatePublishers(self):
        self.point_cloud_publisher = rospy.Publisher(
            f"/drone{self.drone_id}/velodyne_points", PointCloud2)

        self.location_publisher = rospy.Publisher(f"/drone{self.drone_id}/comm/locations", Marker)

    def CreateTimers(self):
        rospy.Timer(rospy.Duration(1 / self.map_sharing_freq), self.GetMergedMaps)
        rospy.Timer(rospy.Duration(1 / self.loc_sharing_freq), self.GetLocations)
        rospy.Timer(rospy.Duration(1), self.CheckForDrones)

    def UpdatePlans(self, event):
        pass

    def UpdateLocations(self, drone_id, msg):
        if drone_id != self.drone_id:
            self.drone_locs[drone_id] = msg.point
        else:
            self.myloc = msg.point

    def UpdateMaps(self, drone_id, msg):
        if drone_id != self.drone_id:
            self.drone_maps[drone_id] = msg

    def GetMergedMaps(self, event):
        for key, map_pcl in self.drone_maps.items():
            if self.dist(self.drone_locs[key]) < self.max_comm_dist and key != self.drone_id:

                generator = pc2.read_points(map_pcl)

                new_point_list = []
                for point in generator:
                    if self.dist(Point(point[0], point[1], point[2])) > 2:
                        new_point_list.append(point)

                new_header = map_pcl.header
                new_header.stamp = rospy.Time.now()
                new_pcl = pc2.create_cloud_xyz32(new_header, new_point_list)
                self.point_cloud_publisher.publish(new_pcl)

    def GetLocations(self, event):
        msg = Marker()
        msg.header.frame_id = "world"
        msg.header.stamp = rospy.Time.now()
        msg.type = msg.SPHERE_LIST
        msg.action = msg.ADD
        msg.scale.x = 0.4
        msg.scale.y = 0.4
        msg.scale.z = 0.4

        msg.color.a = 1.0

        msg.color.r = 0.6
        msg.color.g = 0.0
        msg.color.b = 0.3

        for point in self.drone_locs.values():
            if self.dist(point) < self.max_comm_dist:
                msg.points.append(point)

        self.location_publisher.publish(msg)

    def dist(self, point):
        return sqrt((self.myloc.x - point.x) ** 2 + (self.myloc.y - point.y)**2 + (self.myloc.z - point.z) ** 2)


if __name__ == "__main__":
    name_node = DroneCommNode()
    rospy.spin()
