#ifndef FOUND_BETTER_TARGET_H
#define FOUND_BETTER_TARGET_H

extern RosComm state;

// if the new target is substantially different than the active target, then it has been discovered
BT::NodeStatus TargetDiscovered()
{
    return BT::NodeStatus::SUCCESS;
    ROS_WARN_STREAM("xdist: " << state.active_target.position.x - state.next_target.position.x);
    ROS_WARN_STREAM("ydist: " << state.active_target.position.y - state.next_target.position.y);
    if (std::abs(state.active_target.position.x - state.next_target.position.x) > 0.5 ||
        std::abs(state.active_target.position.y - state.next_target.position.y) > 0.5)
    {

        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

#endif // FOUND_BETTER_TARGET_H