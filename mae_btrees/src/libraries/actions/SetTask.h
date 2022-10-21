#ifndef SET_TASK_H
#define SET_TASK_H

/**
 * @brief Sets a task to the Behavior Tree's blackboard according to the requested task received
 *        from the ros task topic in the BT handler class.
 * @returns Always returns SUCCESS, defaults to Idle.
 */
extern RosComm state;
class SetTask : public BT::SyncActionNode
{
public:
    SetTask(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList
    providedPorts()
    {
        return {BT::OutputPort<std::string>("Task")};
    }

    BT::NodeStatus tick()

    {
        setOutput<std::string>("Task", state.requested_task);
        return BT::NodeStatus::SUCCESS;
    }
};
#endif // SET_TASK_H