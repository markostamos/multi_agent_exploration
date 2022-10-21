#ifndef CLEAR_TASKS_H
#define CLEAR_TASKS_H

extern RosComm state;

/**
 * @brief Sets the current active task of the agent to Idle
 *
 */
class ClearTask : public BT::SyncActionNode
{
public:
    ClearTask(const std::string &name, const BT::NodeConfiguration &config)
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
        state.requested_task = "Idle";
        setOutput<std::string>("Task", "Idle");
        return BT::NodeStatus::SUCCESS;
    }
};
#endif // CLEAR_TASKS_H
