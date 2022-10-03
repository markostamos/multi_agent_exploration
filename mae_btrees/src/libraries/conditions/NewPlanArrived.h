#ifndef NEW_PLAN_ARRIVED_H
#define NEW_PLAN_ARRIVED_H

#include <chrono>
extern RosComm state;

// TODO: CURRENT TARGET??
BT::NodeStatus NewPlanArrived()
{
    static std::vector<geometry_msgs::Point> plan_pts = state.plan_pts;
    static auto now = std::chrono::system_clock::now();

    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count();

    if (state.plan_pts.size() != plan_pts.size() && delta > 5000)
    {
        plan_pts = state.plan_pts;
        now = std::chrono::system_clock::now();
        ROS_WARN_STREAM("New Plan Arrived");
        return BT::NodeStatus::SUCCESS;
    }

    for (int i = 0; i < state.plan_pts.size(); i++)
    {
        if (dist2D(state.plan_pts[i], plan_pts[i]) > 1 && delta > 5000)
        {
            plan_pts = state.plan_pts;
            ROS_WARN_STREAM("New Plan Arrived");
            now = std::chrono::system_clock::now();
            return BT::NodeStatus::SUCCESS;
        }
    }
    return BT::NodeStatus::FAILURE;
}

#endif // NEW_PLAN_ARRIVED_H