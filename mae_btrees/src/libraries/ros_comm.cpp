#include <mae_btrees/ros_comm.h>

extern RosComm state;
void initRosComm(ros::NodeHandle &nh)
{
    state.nh = &nh;
    state.ns = nh.getNamespace().c_str();
    state.publishers.commandPose = nh.advertise<geometry_msgs::PoseStamped>(state.ns + "/command/pose", 100);
    state.publishers.commandVel = nh.advertise<geometry_msgs::Twist>(state.ns + "/command/cmd_vel", 100);
    state.frontier_pts = {};
    state.cancel_goal_req = false;
    state.activity = "idle";
    state.battery_percentage = 100;
}

void waitForConnection()
{

    // Check Publisher connection
    while (state.publishers.commandPose.getNumSubscribers() < 1 ||
           state.publishers.commandVel.getNumSubscribers() < 1)
    {
    }
}