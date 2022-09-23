#ifndef TASK_AVAILABLE_H
#define TASK_AVAILABLE_H

extern RosComm state;

BT::NodeStatus TaskAvailable()
{
    if (state.requested_task == "idle")
        return BT::NodeStatus::FAILURE;
    else
        return BT::NodeStatus::SUCCESS;
}
#endif // TASK_AVAILABLE_H
