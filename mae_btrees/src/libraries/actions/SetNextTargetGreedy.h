#ifndef SET_NEXT_TARGET_GREEDY_H
#define SET_NEXT_TARGET_GREEDY_H

/**
 * @brief Sets the closest frontier as next target to the blackboard.
 * @returns failure if frontier list is empty, success otherwise.
 */
extern RosComm state;
class SetNextTargetGreedy : public BT::SyncActionNode
{
public:
    SetNextTargetGreedy(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus tick()
    {
        if (state.frontier_pts.empty())
            return BT::NodeStatus::FAILURE;

        geometry_msgs::Point target = *std::min_element(state.frontier_pts.begin(), state.frontier_pts.end(), [](const geometry_msgs::Point &a, const geometry_msgs::Point &b)
                                                        { return distFromCurrentPose(a) < distFromCurrentPose(b); });

        geometry_msgs::Pose pose = poseFromVec({target.x, target.y, target.z});

        setOutput<geometry_msgs::Pose>("Target", pose);
        return BT::NodeStatus::SUCCESS;
    }

    inline static float distFromCurrentPose(geometry_msgs::Point pt)
    {
        return sqrt(pow(pt.x - state.pose.position.x, 2) + pow(pt.y - state.pose.position.y, 2) + pow(pt.z - state.pose.position.z, 2));
    }
};
#endif // SET_NEXT_TARGET_GREEDY_H