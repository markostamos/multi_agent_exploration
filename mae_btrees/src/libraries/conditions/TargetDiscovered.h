#ifndef TARGET_DISCOVERED
#define TARGET_DISCOVERED

extern RosComm state;

class TargetDiscovered : public BT::ConditionNode
{
public:
    TargetDiscovered(const std::string &name,
                     const BT::NodeConfiguration &config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus tick() override
    {
        static auto now = std::chrono::system_clock::now();
        geometry_msgs::Pose current_target;
        if (!getInput<geometry_msgs::Pose>("Target", current_target))
        {
            return BT::NodeStatus::SUCCESS;
        }
        if (dist2D(pointFromPose(state.pose), state.plan_pts[0]) < dist2D(state.pose, current_target))
            return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;
    }
};

#endif // TARGET_DISCOVERED