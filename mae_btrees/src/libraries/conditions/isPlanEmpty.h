#ifndef PLAN_EMPTY_H
#define PLAN_EMPTY_H

extern RosComm state;

BT::NodeStatus isPlanEmpty()
{

    return state.plan_pts.empty() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

#endif // PLAN_EMPTY_H
