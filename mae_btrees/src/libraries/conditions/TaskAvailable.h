#ifndef TASK_AVAILABLE_H
#define TASK_AVAILABLE_H

extern RosComm state;

BT::NodeStatus TaskAvailable()
{
    if (state.task_arrived)
    {
        state.task_arrived = false;
        return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
}
#endif // TASK_AVAILABLE_H
