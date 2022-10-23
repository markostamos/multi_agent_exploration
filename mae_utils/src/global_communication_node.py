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
    Simulates global communication for all the agent.
    Location of all agents is known.
    Lidar data is published to a single instance of an octomap_server.
    Dynamically checks for new agent topics based on the agent_handle.
    Interfaces with the global planner to generate plans for agents that are on Exploration Mode.
    """

    def __init__(self):
        rospy.init_node("global_communication_node")
        # Agent topic handle
        self.agent_handle = rospy.get_param("~agent_handle", "drone")
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

        # initialize services
        rospy.wait_for_service("global_planner_node/make_global_plan")
        self.global_plan_service = rospy.ServiceProxy(
            "global_planner_node/make_global_plan", GlobalPlanService, True)

        rospy.Timer(rospy.Duration(0.5), self.checkForAgentTopics)
        rospy.Timer(rospy.Duration(0.1), self.postPCL)
        rospy.Timer(rospy.Duration(2), self.updatePlan)

    def checkForAgentTopics(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find(self.agent_handle)
            if str_loc != -1:
                agent_id = int(topic[0][str_loc + len(self.agent_handle)])
                agent_namespace = self.agent_handle + str(agent_id)
                if agent_id not in self.agent_locations.keys():
                    rospy.Subscriber(
                        f"/{agent_namespace}/ground_truth/position", PointStamped, self.updateLocations, agent_id)

                    rospy.Subscriber(f"/{agent_namespace}/velodyne_points", PointCloud2,
                                     self.updatePCL, agent_id)

                    self.plan_publishers[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/plan", PointArray, queue_size=1000)

                    self.plan_viz_publishers[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/plan_viz", Marker, queue_size=1000)

                    rospy.Subscriber(f"/{agent_namespace}/active_task",
                                     String, self.getActiveTask, agent_id)

                    self.active_tasks[agent_id] = "Idle"
                    self.home_locations[agent_id] = rospy.wait_for_message(
                        f"/{agent_namespace}/ground_truth/position", PointStamped).point

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
        available_agents = [agent_id for agent_id in list(self.agent_locations.keys())
                            if self.active_tasks[agent_id] == "Exploration"]
        targets = self.frontiers.points
        if len(targets) > 0 and len(available_agents) > 0:
            request = GlobalPlanServiceRequest()
            request.starting_positions = PointArray(
                [self.agent_locations[agent_id] for agent_id in available_agents])
            request.targets = PointArray(targets)
            request.timeout_ms = len(available_agents) * (50 if len(targets) < 100 else 100)
            response = self.global_plan_service(request)
            for i in range(len(response.global_plan)):
                self.plan_publishers[available_agents[i]].publish(
                    response.global_plan[i])
                self.publish_plan_visualization(response.global_plan[i], available_agents[i])
        else:
            for agent_id in list(self.agent_locations.keys()):
                self.publish_plan_visualization(PointArray(), agent_id)

    def updatePCL(self, pcl, agent_id):
        self.drone_pcl[agent_id] = pcl

    def postPCL(self, event):
        keys = list(self.drone_pcl.keys())
        for agent_id in keys:
            self.pcl_publisher.publish(self.drone_pcl[agent_id])
            rospy.sleep(0.1)

    def updateLocations(self, msg, agent_id):
        self.agent_locations[agent_id] = msg.point

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

    def publish_plan_visualization(self, plan_array, agent_id):
        marker = Marker()
        color = ((1, 0, 0), (0, 0, 1), (1, 0, 1), (0, 1, 0), (1, 1, 0))[(agent_id - 1) % 4]
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
        self.plan_viz_publishers[agent_id].publish(marker)

    def getActiveTask(self, msg, agent_id):
        if self.active_tasks[agent_id] != msg.data and msg.data == "Exploration":
            self.active_tasks[agent_id] = msg.data
            self.updatePlan()
        else:
            self.active_tasks[agent_id] = msg.data


if __name__ == "__main__":
    rospy.sleep(2)
    name_node = global_communication_node()
    rospy.spin()
