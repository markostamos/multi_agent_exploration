#ifndef SET_NEXT_TARGET_GREEDY_H
#define SET_NEXT_TARGET_GREEDY_H

extern RosComm state;
class SetNextTargetGreedy : public BT::SyncActionNode
{
public:
    SetNextTargetGreedy(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), config_(config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick()
    {

        sort(state.frontier_pts.begin(), state.frontier_pts.end(), SetNextTargetGreedy::compareDistToCurrPose);

        geometry_msgs::Pose pose = poseFromVec({state.frontier_pts[0].x, state.frontier_pts[0].y, state.pose.position.z});

        config_.blackboard->set<geometry_msgs::Pose>("next_target", pose);
        state.next_target = pose;
        return BT::NodeStatus::SUCCESS;
    }

    static bool compareDistToCurrPose(geometry_msgs::Point pt1, geometry_msgs::Point pt2)
    {
        float dist1 = sqrt(pow(pt1.x - state.pose.position.x, 2) + pow(pt1.y - state.pose.position.y, 2));
        float dist2 = sqrt(pow(pt2.x - state.pose.position.x, 2) + pow(pt2.y - state.pose.position.y, 2));
        return dist1 < dist2;
    }

private:
    BT::NodeConfiguration config_;
};
#endif // SET_NEXT_TARGET_GREEDY_H