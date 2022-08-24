#ifndef NEAR_OTHER_DRONES_H
#define NEAR_OTHER_DRONES_H

extern RosComm state;

// if the new target is substantially different than the active target, then it has been discovered
BT::NodeStatus isNearOtherDrones()
{

    for (const auto &pt : state.drone_positions)
    {
        if (dist2D(pointFromPose(state.pose), pt.second) < 3 && (state.ns.back() - '0') > pt.first)
        {
            return BT::NodeStatus::SUCCESS;
        }
    }

    return BT::NodeStatus::FAILURE;
}

#endif // NEAR_OTHER_DRONES_H