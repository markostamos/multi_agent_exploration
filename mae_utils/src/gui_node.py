#!/usr/bin/env python
import rospy
import dearpygui.dearpygui as dpg
from functools import partial
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
import dearpygui.demo as demo


class GUI:
    def __init__(self, app):
        self.ros = app
        self.window_width = 900
        self.window_height = 300

        self.widget_width = 300
        self.initGUI()
        self.create_agent_window(1)
        self.create_agent_window(2)
        self.create_agent_window(3)

    def initGUI(self):
        dpg.create_context()
        dpg.create_viewport(title='Agent Dashboard', width=self.window_width,
                            height=self.window_height)
        dpg.setup_dearpygui()
        dpg.show_viewport()

    def update(self):
        dpg.render_dearpygui_frame()

        # dpg.set_value(f"drone1_pose", f"Drone1 Pose: {self.ros.agent_locations[1]}")
        for drone_id in self.ros.active_tasks.keys():
            dpg.set_value(f"Agent{drone_id}_Active_Task",
                          f"Active Task: {self.ros.active_tasks[drone_id]}")

    def create_agent_window(self, drone_id):
        # demo.show_demo()
        with dpg.window(label=f"Drone{drone_id} commands", width=300, height=300, pos=(300 * (drone_id - 1), 0)):
            with dpg.child_window(label="Behavior Tree Tasks", width=self.widget_width, height=300, pos=(0, 30)):
                dpg.add_button(label="EXPLORATION", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Exploration", drone_id])

                dpg.add_button(label="LAND", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Land", drone_id])
                dpg.add_button(label="TAKE OFF", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["TakeOff", drone_id])

                dpg.add_button(label="GREEDY EXPLORATION", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Greedy_Exploration", drone_id])
                dpg.add_button(label="RETURN HOME", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Return_Home", drone_id])
                dpg.add_button(label="STOP TASK", width=self.widget_width, height=30,
                               callback=self.button_callback, user_data=["Stop", drone_id])
                dpg.add_text("Idle", tag=f"Agent{drone_id}_Active_Task", color=[
                             255, 0, 0])
            """ with dpg.child_window(label="subwindow 1", width=self.widget_width, height=360, pos=(0, 360)):
                dpg.add_input_text(label="GoTo location (x,y,z)",
                                   default_value="0,0,0", tag=f"Agent{drone_id}_GoTo", width=self.widget_width, height=30)

                dpg.add_slider_float(label="float", default_value=0.273, max_value=1)
            with dpg.child_window(label="subwindow 1", width=self.widget_width, height=360, pos=(0, 360 * 2)):
                dpg.add_text(f"Hello, Drone{drone_id}")
                dpg.add_button(label="Save")
                dpg.add_input_text(label="string", default_value="Quick brown fox")
                dpg.add_slider_float(label="float", default_value=0.273, max_value=1) """

    def button_callback(self, sender, app_data, user_data):

        self.ros.task_publishers[user_data[1]].publish(user_data[0])


class GuiNode:
    def __init__(self):
        rospy.init_node("name_node")
        rospy.loginfo("Starting GuiNode as name_node.")
        self.agent_locations = dict()
        self.active_tasks = dict()
        self.task_publishers = dict()

        self.agent_locations[0] = PointStamped()

        rospy.Timer(rospy.Duration(0.5), self.checkForAgentTopics)
        self.checkForAgentTopics(None)

    def checkForAgentTopics(self, event):
        for topic in rospy.get_published_topics():
            str_loc = topic[0].find("drone")
            if str_loc != -1:
                drone_id = int(topic[0][str_loc + len("drone")])
                if drone_id not in self.agent_locations.keys():
                    rospy.Subscriber(
                        f"/drone{drone_id}/ground_truth/position", PointStamped, self.getLocations, drone_id)

                    rospy.Subscriber(f"/drone{drone_id}/active_task",
                                     String, self.getActiveTask, drone_id)

                    self.active_tasks[drone_id] = "Idle"
                    self.task_publishers[drone_id] = rospy.Publisher(
                        f"/drone{drone_id}/task", String, queue_size=10)

    def getLocations(self, msg, drone_id):
        self.agent_locations[drone_id] = msg.point

    def getActiveTask(self, msg, drone_id):
        self.active_tasks[drone_id] = msg.data


if __name__ == "__main__":
    app = GuiNode()

    rospy.sleep(2)
    gui = GUI(app)

    # below replaces, start_dearpygui()
    while dpg.is_dearpygui_running() and not rospy.is_shutdown():

        gui.update()

        rospy.sleep(0.01)

    dpg.destroy_context()
