#!/usr/bin/env python
import rospy
import dearpygui.dearpygui as dpg
from geometry_msgs.msg import PointStamped, PoseStamped, Twist, Pose, Point
from std_msgs.msg import String
from gazebo_msgs.msg import ModelState
import rosservice
from std_srvs.srv import Empty


class GUI:
    def __init__(self, app):
        self.ros = app
        self.window_width = 2000
        self.window_height = 600
        self.widget_width = 400
        self.agent_windows = []

        self.initGUI()

    def initGUI(self):
        dpg.create_context()
        dpg.create_viewport(title='Agent Dashboard', width=self.window_width,
                            height=self.window_height)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def update(self):
        for agent_id in list(self.ros.agent_locations.keys()):
            if agent_id not in self.agent_windows:
                self.create_agent_window(agent_id)
                self.agent_windows.append(agent_id)
                dpg.set_viewport_width(self.widget_width * len(self.agent_windows))

        for agent_id in self.ros.active_tasks.keys():
            dpg.set_value(f"Agent{agent_id}_Active_Task",
                          f"Active Task: {self.ros.active_tasks[agent_id]}")
        for agent_id in self.ros.agent_locations.keys():
            agent_pos = self.ros.agent_locations[agent_id]

            pos_string = f"Position: ({agent_pos.x:.2f}, {agent_pos.y:.2f}, {agent_pos.z:.2f})"

            dpg.set_value(f"Agent{agent_id}_Position", pos_string)
        dpg.render_dearpygui_frame()

    def create_agent_window(self, agent_id):
        # MISSIONS WINDOW
        with dpg.window(label=f"Agent{agent_id} commands", width=self.widget_width, height=300, pos=(self.widget_width * (agent_id - 1), 0)):
            with dpg.group(label="Behavior Tree Tasks"):
                dpg.add_button(label="EXPLORATION", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Exploration", agent_id])

                dpg.add_button(label="LAND", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Land", agent_id])
                dpg.add_button(label="TAKE OFF", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["TakeOff", agent_id])

                dpg.add_button(label="GREEDY EXPLORATION", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Greedy_Exploration", agent_id])
                dpg.add_button(label="RETURN HOME", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Return_Home", agent_id])
                dpg.add_button(label="STOP TASK", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Stop", agent_id])
                dpg.add_button(label="CLEAR MAP", width=self.widget_width, height=30,
                               callback=self.clearMapCallback, user_data=agent_id)
                dpg.add_text("Idle", tag=f"Agent{agent_id}_Active_Task", color=[
                             255, 0, 0], indent=110)
        # GOTO WINDOW
        with dpg.window(label=f"Agent{agent_id} GoTo", width=self.widget_width, height=150, pos=(self.widget_width * (agent_id - 1), 300)):
            with dpg.group(horizontal=True):
                width = self.widget_width / 2 - 15
                with dpg.group():
                    dpg.add_input_float(label="x", width=width, step=0, tag=f"Agent{agent_id}_x")
                    dpg.add_input_float(label="y", width=width, step=0, tag=f"Agent{agent_id}_y")
                    dpg.add_input_float(label="z", width=width, step=0, tag=f"Agent{agent_id}_z")
                dpg.add_button(label="GoTo", height=65, width=width,
                               callback=self.goToCallback, user_data=agent_id)

            dpg.add_text("Position: ", tag=f"Agent{agent_id}_Position", indent=85)

        # MANUAL CONTROL WINDOW
        with dpg.window(label=f"Agent{agent_id} Manual Control", width=self.widget_width, height=250, pos=(self.widget_width * (agent_id - 1), 450)):
            with dpg.handler_registry():
                # dpg.add_checkbox(label="Keyboard Control", tag="keyboard_control")
                dpg.add_key_press_handler(callback=self.keyCallback, user_data=agent_id)
            dpg.add_slider_float(label="Speed", width=self.widget_width - 50,
                                 min_value=0, max_value=100, default_value=0.5, tag=f"Agent{agent_id}_speed")

            dpg.add_text("W,A,S,D for movement in x,y plane", indent=50)
            dpg.add_text("Up,Down for movement in z plane", indent=50)

    def keyCallback(self, sender, app_data, agent_id):
        self.ros.publishTask(agent_id, "Idle")
        speed = dpg.get_value(f"Agent{agent_id}_speed") * 2 / 100
        target = [0, 0, 0]
        if app_data == dpg.mvKey_W:
            target[1] = speed
        elif app_data == dpg.mvKey_A:
            target[0] = -speed
        elif app_data == dpg.mvKey_S:
            target[1] = -speed
        elif app_data == dpg.mvKey_D:
            target[0] = speed
        elif app_data == dpg.mvKey_Up:
            target[2] = speed
        elif app_data == dpg.mvKey_Down:
            target[2] = -speed

        self.ros.publishManualControl(agent_id=agent_id, target=target)

    def goToCallback(self, sender, app_data, agent_id):
        self.ros.publishTask(agent_id, "Idle")
        x = dpg.get_value(f"Agent{agent_id}_x")
        y = dpg.get_value(f"Agent{agent_id}_y")
        z = dpg.get_value(f"Agent{agent_id}_z")
        if z == 0:
            self.ros.publishGoal(agent_id, (x, y, z))
        else:
            self.ros.publish3DGoal(agent_id, (x, y, z))

    def button_callback(self, sender, app_data, user_data):
        self.ros.publishTask(user_data[1], user_data[0])

    def clearMapCallback(self, sender, app_data, agent_id):
        self.ros.publishClearMap(agent_id)


class GuiNode:
    def __init__(self):
        rospy.init_node("name_node")
        rospy.loginfo("Starting GuiNode as name_node.")
        self.agent_locations = dict()
        self.active_tasks = dict()
        self.task_publishers = dict()
        self.goTo_publishers = dict()
        self.goTo_publishers3D = dict()
        self.agent_locations = dict()
        self.cmdvel_publishers = dict()

        self.agent_handle = rospy.get_param("~agent_handle", "drone")
        rospy.Timer(rospy.Duration(0.5), self.checkForAgentTopics)
        self.checkForAgentTopics(None)

    def checkForAgentTopics(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find(self.agent_handle)
            if str_loc != -1:
                agent_id = int(topic[0][str_loc + len(self.agent_handle)])
                agent_namespace = str(self.agent_handle) + str(agent_id)
                if agent_id not in self.agent_locations.keys():
                    rospy.Subscriber(
                        f"/{agent_namespace}/ground_truth/position", PointStamped, self.getLocations, agent_id)

                    rospy.Subscriber(f"/{agent_namespace}/active_task",
                                     String, self.getActiveTask, agent_id)

                    self.active_tasks[agent_id] = "Idle"
                    self.task_publishers[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/task", String, queue_size=10)
                    self.goTo_publishers[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/move_base_simple/goal", PoseStamped, queue_size=10)
                    self.goTo_publishers3D[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/move_base_3d_simple/goal", Point, queue_size=10)
                    self.cmdvel_publishers[agent_id] = rospy.Publisher(
                        f"/{agent_namespace}/command/cmd_vel", Twist, queue_size=10)

    def getLocations(self, msg, agent_id):
        self.agent_locations[agent_id] = msg.point

    def getActiveTask(self, msg, agent_id):
        self.active_tasks[agent_id] = msg.data

    def publishTask(self, agent_id, task):
        self.task_publishers[agent_id].publish(task)

    def publishGoal(self, agent_id, goal_xyz):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = dpg.get_value(f"Agent{agent_id}_x")
        pose.pose.position.y = dpg.get_value(f"Agent{agent_id}_y")
        pose.pose.position.z = dpg.get_value(f"Agent{agent_id}_z")
        pose.pose.orientation.w = 1
        self.goTo_publishers[agent_id].publish(pose)

    def publish3DGoal(self, agent_id, goal_xyz):
        point = Point()
        point.x = goal_xyz[0]
        point.y = goal_xyz[1]
        point.z = goal_xyz[2]
        self.goTo_publishers3D[agent_id].publish(point)

    def publishManualControl(self, agent_id, target):
        msg = Twist()

        msg.linear.x = target[0]
        msg.linear.y = target[1]
        msg.linear.z = target[2]

        self.cmdvel_publishers[agent_id].publish(msg)

    def publishClearMap(self, agent_id):
        service_name = f"/{self.agent_handle + str(agent_id)}/octomap_server/reset"
        if service_name in rosservice.get_service_list():
            rospy.wait_for_service(service_name)
            try:
                reset_octomap = rospy.ServiceProxy(service_name, Empty)
                reset_octomap()
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)
        else:
            rospy.wait_for_service("/octomap_server/reset")
            try:
                reset_octomap = rospy.ServiceProxy("/octomap_server/reset", Empty)
                reset_octomap()
            except rospy.ServiceException as e:
                print("Service call failed: %s" % e)


if __name__ == "__main__":
    app = GuiNode()

    rospy.sleep(2)
    gui = GUI(app)

    # below replaces, start_dearpygui()
    while dpg.is_dearpygui_running() and not rospy.is_shutdown():

        gui.update()

        rospy.sleep(0.01)

    dpg.destroy_context()
