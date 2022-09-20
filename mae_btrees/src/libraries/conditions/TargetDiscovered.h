#ifndef TARGET_DISCOVERED
#define TARGET_DISCOVERED

extern RosComm state;

// if the new target is substantially different than the active target, then it has been discovered
BT::NodeStatus TargetDiscovered()
{
    if (state.plan_pts.empty())
    {
        return BT::NodeStatus::FAILURE;
    }

    bool pose_close_to_target = dist2D(pointFromPose(state.next_target), pointFromPose(state.pose)) < 1;
    bool better_plan_0 = dist2D(pointFromPose(state.pose), state.plan_pts[0]) < dist2D(pointFromPose(state.pose), pointFromPose(state.next_target));

    bool target_not_in_frontiers = std::find_if(state.frontier_pts.begin(), state.frontier_pts.end(), [&](const geometry_msgs::Point &p)
                                                { return dist2D(p, state.plan_pts[0]) < 1; }) == state.frontier_pts.end();
    if (pose_close_to_target || better_plan_0 || target_not_in_frontiers)
    {
        if (!state.replan)
            state.plan_pts.erase(state.plan_pts.begin());
        state.replan = false;

        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}

#endif // TARGET_DISCOVERED