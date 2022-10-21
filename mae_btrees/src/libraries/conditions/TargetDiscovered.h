#ifndef TARGET_DISCOVERED
#define TARGET_DISCOVERED

extern RosComm state;

/**
 * @brief Checks whether the current active target exists in the list of frontier points.
 * @returns Success if the target is not in the list, failure otherwise.
 */
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
            return BT::NodeStatus::SUCCESS;

        if (state.plan_pts.empty())
            return BT::NodeStatus::FAILURE;

        for (const auto &pt : state.frontier_pts)
        {
            if (dist3D(pointFromPose(current_target), pt) < 2)
                return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // TARGET_DISCOVERED