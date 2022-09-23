#ifndef CLEAR_TASKS_H
#define CLEAR_TASKS_H

extern RosComm state;
extern RosComm state;
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
        setOutput<std::string>("Task", "");
        return BT::NodeStatus::SUCCESS;
    }
};
#endif // CLEAR_TASKS_H
