#ifndef NEXT_TARGET_FROM_PLAN_H
#define NEXT_TARGET_FROM_PLAN_H

/* Set Locations to blackboard by name */
extern RosComm state;
class SetNextTargetFromPlan : public BT::SyncActionNode
{
public:
    SetNextTargetFromPlan(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::OutputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus tick()
    {
        if (!state.plan_pts.empty())
        {
            geometry_msgs::Pose pose = poseFromVec({state.plan_pts[0].x, state.plan_pts[0].y, state.pose.position.z});
            setOutput("Target", pose);
            return BT::NodeStatus::SUCCESS;
        }

        return BT::NodeStatus::FAILURE;
    }
};
#endif // NEXT_TARGET_FROM_PLAN_H
