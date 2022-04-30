#ifndef CANCEL_GOAL_H
#define CANCEL_GOAL_H

/* Set Locations to blackboard by name */
extern RosComm state;
BT::NodeStatus CancelGoal()
{
    state.cancel_goal_req = true;
    return BT::NodeStatus::SUCCESS;
}
#endif // CANCEL_GOAL_H
