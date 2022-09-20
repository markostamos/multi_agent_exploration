#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped, Point
from nav_msgs.msg import OccupancyGrid
from mae_global_planner.srv import PlanService, PlanServiceRequest, PlanServiceResponse
from mae_global_planner.srv import GlobalPlanService, GlobalPlanServiceRequest, GlobalPlanServiceResponse
from visualization_msgs.msg import MarkerArray, Marker
from mae_utils.msg import PointArray
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

        self.id = int(rospy.get_namespace()[-2])
        self.max_comm_dist = rospy.get_param("~max_comm_dist", 100)
        self.ids = []
        self.locations = dict()
        self.pcl_centers = dict()
        self.lidar_readings = dict()
        self.connected = dict()
        self.frontiers = PointArray()
        self.plan_publishers = dict()

        self.initCommunication()
        self.createTimers()

    def initCommunication(self):

        # initialize subscribers
        rospy.Subscriber(f"/drone{self.id}/projected_map",
                         OccupancyGrid, self.filterMap)
        rospy.Subscriber(f"/drone{self.id}/frontiers", PointArray, self.getFrontiers)
        rospy.Subscriber(f"/drone{self.id}/plan", PointArray, self.visualizePlan)

        # initialize publishers
        self.pcl_publisher = rospy.Publisher(
            f"/drone{self.id}/combined_pcl", PointCloud2, queue_size=100)
        self.filtered_map_publisher = rospy.Publisher(
            f"/drone{self.id}/filtered_map", OccupancyGrid, queue_size=1000)
        self.marker_publisher = rospy.Publisher(
            f"/drone{self.id}/markers", Marker, queue_size=1000)
        # initialize services
        rospy.wait_for_service("make_plan")
        rospy.wait_for_service("make_global_plan")
        rospy.loginfo("Services are available")
        self.plan_service = rospy.ServiceProxy(
            "make_plan", PlanService, persistent=True)
        self.global_plan_service = rospy.ServiceProxy("make_global_plan", GlobalPlanService, True)
        self.checkForNewTopics(event=None)

    def checkForNewTopics(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find("drone")
            if str_loc != -1:
                drone_id = int(topic[0][str_loc + len("drone")])
                if drone_id not in self.ids:
                    self.ids.append(drone_id)
                    rospy.Subscriber(
                        f"/drone{drone_id}/ground_truth/position", PointStamped, self.getLocations, drone_id)
                    rospy.Subscriber(f"/drone{drone_id}/octomap_point_cloud_centers",
                                     PointCloud2, self.getCentersPCL, drone_id)
                    rospy.Subscriber(f"/drone{drone_id}/velodyne_points", PointCloud2,
                                     self.getLidarReadings, drone_id)

                    self.plan_publishers[drone_id] = rospy.Publisher(
                        f"/drone{drone_id}/plan", PointArray, queue_size=1000)
                    self.connected[drone_id] = True if drone_id == self.id else False
                    self.lidar_readings[drone_id] = PointCloud2()
                    self.pcl_centers[drone_id] = PointCloud2()
                    self.locations[drone_id] = Point()
                    self.updatePlan()

    def getFrontiers(self, msg):

        def significant_change(): return any(
            [self.dist(msg.points[i], self.frontiers.points[i]) > 0.5 for i in range(len(msg.points))])

        msg.points.sort(key=lambda p: self.dist(p, Point(0, 0, 0)))
        if len(msg.points) != len(self.frontiers.points) or significant_change():
            self.frontiers = msg
            self.updatePlan()

    def createTimers(self):
        rospy.Timer(rospy.Duration(3), self.checkForNewTopics)
        rospy.Timer(rospy.Duration(0.1), self.checkConnections)

        rospy.Timer(rospy.Duration(0.1), self.shareLidarReadings)

    def updatePlan(self):
        connected_ids = [drone_id for drone_id in self.ids if self.connected[drone_id]]
        targets = self.frontiers.points
        if len(targets) > 0 and self.id == min(connected_ids):

            if len(connected_ids) == 1:
                request = PlanServiceRequest()
                request.starting_position = self.locations[self.id]
                request.targets = PointArray(targets)
                request.timeout_ms = 100
                response = self.plan_service(request)
                self.plan_publishers[self.id].publish(response.plan)
            else:
                request = GlobalPlanServiceRequest()
                request.starting_positions = PointArray(
                    [self.locations[drone_id] for drone_id in connected_ids])
                request.targets = PointArray(targets)
                request.timeout_ms = len(connected_ids) * (50 if len(targets) < 100 else 100)
                response = self.global_plan_service(request)
                for i in range(len(connected_ids)):
                    self.plan_publishers[connected_ids[i]].publish(
                        response.global_plan[i])

    def visualizePlan(self, msg):
        marker = Marker()
        color = ((0, 0, 0), (1, 0, 0), (0, 0.5, 0), (1, 0, 0), (1, 1, 0))[(self.id - 1) % 4]
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.points = msg.points
        self.marker_publisher.publish(marker)

    def shareLidarReadings(self, event):
        for drone_id in self.ids:
            if self.connected[drone_id]:
                self.pcl_publisher.publish(self.lidar_readings[drone_id])
            rospy.sleep(0.1)

    def checkConnections(self, event):
        connection_status_updated = False
        for drone_id in self.ids:
            if self.dist(self.locations[self.id], self.locations[drone_id]) < self.max_comm_dist:
                if self.connected[drone_id] == False:
                    self.connected[drone_id] = True
                    rospy.logwarn(
                        f"Drone {self.id} connected to drone {drone_id} after a period of disconnectedness")
                    self.mapMerge(drone_id)
                    connection_status_updated = True
            else:
                self.connected[drone_id] = False

        if connection_status_updated:
            self.updatePlan()

    def mapMerge(self, drone_id):
        rospy.logwarn(f"Drone {self.id} getting map updates from {drone_id}")
        self.pcl_publisher.publish(self.pcl_centers[drone_id])

    def getLocations(self, msg, drone_id):
        self.locations[drone_id] = msg.point

    def getCentersPCL(self, msg, drone_id):
        self.pcl_centers[drone_id] = msg

    def filterMap(self, map_msg):
        map_msg.data = list(map_msg.data)

        point = self.locations[self.id]
        grid_x = int((point.x - map_msg.info.origin.position.x) / map_msg.info.resolution)
        grid_y = int((point.y - map_msg.info.origin.position.y) / map_msg.info.resolution)
        for i in range(grid_x - 3, grid_x + 4):
            for j in range(grid_y - 3, grid_y + 4):
                if int(i + j * map_msg.info.width) < len(map_msg.data):
                    map_msg.data[int(i + j * map_msg.info.width)] = 0

        self.filtered_map_publisher.publish(map_msg)

    def getLidarReadings(self, msg, drone_id):
        self.lidar_readings[drone_id] = msg

    def dist(self, p1, p2):
        return sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2)


if __name__ == "__main__":
    name_node = DroneCommNode()
    rospy.spin()
