#ifndef TARGET_FOUND
#define TARGET_FOUND

extern RosComm state;

class TargetFound : public BT::ConditionNode
{
public:
    TargetFound(const std::string &name,
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
            return BT::NodeStatus::SUCCESS;

        if (state.plan_pts.empty())
            return BT::NodeStatus::FAILURE;

        for (const auto &pt : state.frontier_pts)
        {
            if (dist2D(pointFromPose(current_target), pt) < 2)
                return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // TARGET_FOUND