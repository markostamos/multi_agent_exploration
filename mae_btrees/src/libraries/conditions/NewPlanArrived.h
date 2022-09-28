#ifndef NEW_PLAN_ARRIVED_H
#define NEW_PLAN_ARRIVED_H

#include <chrono>
extern RosComm state;

BT::NodeStatus NewPlanArrived()
{
    /* static std::vector<geometry_msgs::Point> plan_pts = state.plan_pts;
    static auto now = std::chrono::system_clock::now();

    auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - now).count();

    if (delta > 1500)
    {

        if (plan_pts.size() != state.plan_pts.size())
        {

            plan_pts = state.plan_pts;
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            for (int i = 0; i < plan_pts.size(); i++)
            {
                if (plan_pts[i].x != state.plan_pts[i].x || plan_pts[i].y != state.plan_pts[i].y || plan_pts[i].z != state.plan_pts[i].z)
                {
                    plan_pts = state.plan_pts;
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }

        now = std::chrono::system_clock::now();
    } */
    return BT::NodeStatus::FAILURE;
}

#endif // NEW_PLAN_ARRIVED_H
