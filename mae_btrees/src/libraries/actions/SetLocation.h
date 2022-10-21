#ifndef SET_LOCATION_H
#define SET_LOCATION_H

#include <geometry_msgs/Pose.h>

/**
 * @brief Sets a location to the Behavior Trees blackboard by name.
 *        The location can later be used as a target for GoTo actions.
 *          Target = {NAME} in XML tree.
 *
 */
class SetLocation : public BT::SyncActionNode
{
public:
    SetLocation(const std::string &name, const BT::NodeConfiguration &config)
        : BT::SyncActionNode(name, config), config_(config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::Pose>("Pos"),
                BT::InputPort<std::string>("Name")};
    }

    BT::NodeStatus tick() override
    {
        geometry_msgs::Pose pose;
        std::string name;
        if (!getInput<geometry_msgs::Pose>("Pos", pose) || !getInput<std::string>("Name", name))
        {
            throw BT::RuntimeError("Missing input x,y");
        }

        config_.blackboard->set<geometry_msgs::Pose>(name, poseFromVec({pose.position.x, pose.position.y, pose.position.z}));

        return BT::NodeStatus::SUCCESS;
    }

private:
    BT::NodeConfiguration config_;
};

#endif // SET_LOCATION_H