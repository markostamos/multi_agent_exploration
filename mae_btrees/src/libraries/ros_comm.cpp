#include <mae_btrees/ros_comm.h>

extern RosComm state;
void initRosComm(ros::NodeHandle &nh)
{
    state.nh = &nh;
    state.ns = nh.getNamespace().c_str();
    state.frontier_pts = {};

    state.requested_task = "Idle";
    state.task_arrived = true;
    state.battery_percentage = 100;
}
