#ifndef GREEDY_TARGET_DISCOVERED_H
#define GREEDY_TARGET_DISCOVERED_H

extern RosComm state;

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
            if (dist2D(pointFromPose(current_target), pt) < 2)
            {
                return BT::NodeStatus::FAILURE;
            }
        }
        return BT::NodeStatus::SUCCESS;
    }
};

#endif // GREEDY_TARGET_DISCOVERED_H