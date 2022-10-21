#ifndef GREEDY_TARGET_DISCOVERED_H
#define GREEDY_TARGET_DISCOVERED_H

extern RosComm state;
/**
 * @brief Checks whether the current active target exists in the list of frontier points.
 *        If not it assumes it has been discovered and returns success.
 */
class GreedyTargetDiscovered : public BT::ConditionNode
{
public:
    GreedyTargetDiscovered(const std::string &name,
                           const BT::NodeConfiguration &config) : BT::ConditionNode(name, config) {}

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::Pose current_target;
        if (!getInput<geometry_msgs::Pose>("Target", current_target))
        {
            return BT::NodeStatus::SUCCESS;
        }
        for (const auto &pt : state.frontier_pts)
        {
            if (dist3D(pointFromPose(current_target), pt) < 2)
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // GREEDY_TARGET_DISCOVERED_H