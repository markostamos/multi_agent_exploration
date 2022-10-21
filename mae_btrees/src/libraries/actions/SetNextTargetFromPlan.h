#ifndef NEXT_TARGET_FROM_PLAN_H
#define NEXT_TARGET_FROM_PLAN_H

/**
 * @brief Gets the first target from the plan and sets it to the blackboard.
 * @returns failure if plan is empty, success otherwise.
 */
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
        geometry_msgs::Pose current_target;

        if (state.plan_pts.empty())
            return BT::NodeStatus::FAILURE;

        geometry_msgs::Pose pose = poseFromVec({state.plan_pts[0].x, state.plan_pts[0].y, 0});
        setOutput("Target", pose);
        return BT::NodeStatus::SUCCESS;
    }
};
#endif // NEXT_TARGET_FROM_PLAN_H
