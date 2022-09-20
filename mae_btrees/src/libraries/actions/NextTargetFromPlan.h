#ifndef NEXT_TARGET_FROM_PLAN_H
#define NEXT_TARGET_FROM_PLAN_H

/* Set Locations to blackboard by name */
extern RosComm state;
class NextTargetFromPlan : public BT::SyncActionNode
{
public:
    NextTargetFromPlan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), config_(config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick()
    {
        ROS_WARN_STREAM("plan size: " << state.plan_pts.size());
        if (!state.plan_pts.empty())
        {
            ROS_WARN_STREAM("PLAN len " << state.plan_pts.size());
            geometry_msgs::Pose pose = poseFromVec({state.plan_pts[0].x, state.plan_pts[0].y, state.pose.position.z});
            config_.blackboard->set<geometry_msgs::Pose>("next_target", pose);
            state.next_target = pose;
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    BT::NodeConfiguration config_;
};
#endif // NEXT_TARGET_FROM_PLAN_H
