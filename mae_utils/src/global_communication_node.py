#!/usr/bin/env python
import rospy

from sensor_msgs.msg import PointCloud2
from mae_utils.msg import PointArray
from mae_global_planner.srv import GlobalPlanService, GlobalPlanServiceRequest, GlobalPlanServiceResponse
from visualization_msgs.msg import Marker
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import String


class global_communication_node:
    """
    This node subscribes to the lidar sensors of all the uavs and
    republishes the point clouds to a single topic so that a global
    map can be made.
    """

    def __init__(self):
        rospy.init_node("global_communication_node")

        # initialize variables
        self.agent_locations = dict()
        self.home_locations = dict()
        self.drone_pcl = dict()
        self.frontiers = PointArray()
        self.active_tasks = dict()

        # initialize subscribers
        rospy.Subscriber('/projected_map', OccupancyGrid, self.filterMap)
        rospy.Subscriber('/frontiers', PointArray, self.updateFrontiers)

        # initialize publishers
        self.pcl_publisher = rospy.Publisher(
            '/combined_pcl', PointCloud2, queue_size=100)
        self.map_publisher = rospy.Publisher('/filtered_map', OccupancyGrid, queue_size=1000)
        self.plan_publishers = dict()
        self.plan_viz_publishers = dict()

        self.checkpoint_publisher = rospy.Publisher("/checkpoints", PointArray, queue_size=1000)

        # initialize services
        rospy.wait_for_service("global_planner_node/make_global_plan")
        self.global_plan_service = rospy.ServiceProxy(
            "global_planner_node/make_global_plan", GlobalPlanService, True)

        rospy.Timer(rospy.Duration(0.5), self.checkForDrones)
        rospy.Timer(rospy.Duration(0.1), self.postPCL)
        rospy.Timer(rospy.Duration(2), self.updatePlan)
        rospy.Timer(rospy.Duration(2), self.publishCheckpoints)

    def checkForDrones(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find("drone")
            if str_loc != -1:
                drone_id = int(topic[0][str_loc + len("drone")])
                if drone_id not in self.agent_locations.keys():
                    """ self.agent_locations["home" + str(drone_id)] = rospy.wait_for_message(
                        f"/drone{drone_id}/ground_truth/position", PointStamped).point """
                    rospy.Subscriber(
                        f"/drone{drone_id}/ground_truth/position", PointStamped, self.updateLocations, drone_id)

                    rospy.Subscriber(f"/drone{drone_id}/velodyne_points", PointCloud2,
                                     self.updatePCL, drone_id)

                    self.plan_publishers[drone_id] = rospy.Publisher(
                        f"/drone{drone_id}/plan", PointArray, queue_size=1000)

                    self.plan_viz_publishers[drone_id] = rospy.Publisher(
                        f"/drone{drone_id}/plan_viz", Marker, queue_size=1000)

                    rospy.Subscriber(f"/drone{drone_id}/active_task",
                                     String, self.getActiveTask, drone_id)

                    self.active_tasks[drone_id] = "Idle"
                    self.home_locations[drone_id] = rospy.wait_for_message(
                        f"/drone{drone_id}/ground_truth/position", PointStamped).point

    def updateFrontiers(self, msg):
        def significant_change(): return any(
            [self.dist(msg.points[i], self.frontiers.points[i]) > 0.2 for i in range(len(msg.points))])

        msg.points.sort(key=lambda p: self.dist(p, Point(0, 0, 0)))
        if len(msg.points) != len(self.frontiers.points):
            self.frontiers = msg
            self.updatePlan()
        elif significant_change():
            self.frontiers = msg
            self.updatePlan()
        else:
            self.frontiers = msg

    def updatePlan(self, event=None):
        rospy.logerr("Updating Plan")
        available_agents = [drone_id for drone_id in list(self.agent_locations.keys())
                            if self.active_tasks[drone_id] == "Exploration"]
        targets = self.frontiers.points
        if len(targets) > 0 and len(available_agents) > 0:
            request = GlobalPlanServiceRequest()
            request.starting_positions = PointArray(
                [self.agent_locations[drone_id] for drone_id in available_agents])
            request.targets = PointArray(targets)
            request.timeout_ms = len(available_agents) * (50 if len(targets) < 100 else 100)
            response = self.global_plan_service(request)
            for i in range(len(response.global_plan)):
                self.plan_publishers[available_agents[i]].publish(
                    response.global_plan[i])
                self.publish_plan_visualization(response.global_plan[i], available_agents[i])

    def updatePCL(self, pcl, drone_id):
        self.drone_pcl[drone_id] = pcl

    def postPCL(self, event):
        keys = list(self.drone_pcl.keys())
        for drone_id in keys:
            self.pcl_publisher.publish(self.drone_pcl[drone_id])
            rospy.sleep(0.1)

    def updateLocations(self, msg, drone_id):
        self.agent_locations[drone_id] = msg.point

    def dist(self, point1, point2):
        return ((point1.x - point2.x)**2 + (point1.y - point2.y)**2 + (point1.z - point2.z)**2)**0.5

    def filterMap(self, map_msg):
        map_msg.data = list(map_msg.data)

        for point in list(self.agent_locations.values()) + list(self.home_locations.values()):
            # point to 2dmap index
            grid_x = int((point.x - map_msg.info.origin.position.x) / map_msg.info.resolution)
            grid_y = int((point.y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            for i in range(grid_x - 3, grid_x + 4):
                for j in range(grid_y - 3, grid_y + 4):
                    if int(i + j * map_msg.info.width) < len(map_msg.data):
                        map_msg.data[int(i + j * map_msg.info.width)] = 0

        self.map_publisher.publish(map_msg)

    def publish_plan_visualization(self, plan_array, drone_id):
        marker = Marker()
        color = ((1, 0, 0), (0, 0, 1), (1, 0, 1), (0, 1, 0), (1, 1, 0))[(drone_id - 1) % 4]
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.points = plan_array.points
        self.plan_viz_publishers[drone_id].publish(marker)

    def getActiveTask(self, msg, drone_id):
        if self.active_tasks[drone_id] != msg.data and msg.data == "Exploration":
            self.active_tasks[drone_id] = msg.data
            self.updatePlan()
        else:
            self.active_tasks[drone_id] = msg.data

    def publishCheckpoints(self, event):
        msg = PointArray()
        msg.points = list(self.agent_locations.values())
        self.checkpoint_publisher.publish(msg)


if __name__ == "__main__":
    rospy.sleep(2)
    name_node = global_communication_node()
    rospy.spin()
