#ifndef GREEDY_TARGET_DISCOVERED_H
#define GREEDY_TARGET_DISCOVERED_H

extern RosComm state;

// if the new target is substantially different than the active target, then it has been discovered
BT::NodeStatus GreedyTargetDiscovered()
{
    for (const auto &pt : state.frontier_pts)
    {
        if (dist2D(pointFromPose(state.next_target), pt) < 2)
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    return BT::NodeStatus::SUCCESS;
}

#endif // GREEDY_TARGET_DISCOVERED_H