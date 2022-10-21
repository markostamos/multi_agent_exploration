#ifndef NEW_PLAN_ARRIVED_H
#define NEW_PLAN_ARRIVED_H

#include <chrono>
extern RosComm state;

/**
 * @brief Checks whether a new plan has arrived every 5 seconds.
 * @returns Success if a new plan has arrived in order to break from a Fallback control node and interrupt current target.
 */
BT::NodeStatus NewPlanArrived()
{
    static std::vector<geometry_msgs::Point> plan_pts = state.plan_pts;
    static auto now = std::chrono::system_clock::now();

    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count();

    if (state.plan_pts.empty())
        return BT::NodeStatus::FAILURE;

    if (state.plan_pts.size() != plan_pts.size() && delta > 5000)
    {
        plan_pts = state.plan_pts;
        now = std::chrono::system_clock::now();
        ROS_WARN_STREAM("New Plan Arrived");
        return BT::NodeStatus::SUCCESS;
    }

    for (int i = 0; i < state.plan_pts.size(); i++)
    {
        if (dist3D(state.plan_pts[i], plan_pts[i]) > 1 && delta > 5000)
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