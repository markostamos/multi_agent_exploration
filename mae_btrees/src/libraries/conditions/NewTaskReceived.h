#ifndef NEW_TASK_RECEIVED_H
#define NEW_TASK_RECEIVED_H

extern RosComm state;

// if the new target is substantially different than the active target, then it has been discovered
BT::NodeStatus NewTaskReceived()
{
    if (state.activity == "idle")
        return BT::NodeStatus::FAILURE;
    else
        return BT::NodeStatus::SUCCESS;
}
#endif // NEW_TASK_RECEIVED_H
