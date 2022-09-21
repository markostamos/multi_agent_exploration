#ifndef SET_TASK_H
#define SET_TASK_H
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
        setOutput<std::string>("Task", state.activity);
        return BT::NodeStatus::SUCCESS;
    }
};
#endif // SET_TASK_H