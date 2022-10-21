#ifndef BETTER_TARGET_FOUND
#define BETTER_TARGET_FOUND

extern RosComm state;

/**
 * @brief Checks if the suggested target from the current plan is better than the active current target.
 * @returns SUCCESS if this statement holds , FAILURE otherwise.
 */
class BetterTargetFound : public BT::ConditionNode
{
public:
    BetterTargetFound(const std::string &name,
                      const BT::NodeConfiguration &config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus tick() override
    {
        static auto now = std::chrono::system_clock::now();
        geometry_msgs::Pose current_target;

        if (state.plan_pts.empty())
            return BT::NodeStatus::FAILURE;

        if (!getInput<geometry_msgs::Pose>("Target", current_target))
        {
            return BT::NodeStatus::SUCCESS;
        }
        if (dist3D(pointFromPose(state.pose), state.plan_pts[0]) < dist3D(state.pose, current_target))
            return BT::NodeStatus::SUCCESS;

        return BT::NodeStatus::FAILURE;
    }
};

#endif // BETTER_TARGET_FOUND