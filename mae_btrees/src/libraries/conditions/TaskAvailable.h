#ifndef TASK_AVAILABLE_H
#define TASK_AVAILABLE_H

extern RosComm state;

/**
 * @brief Checks whether a new task has arrived, used to interrupt current non safety task in a Fallback control node.
 *
 * @return BT::NodeStatus
 */
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
