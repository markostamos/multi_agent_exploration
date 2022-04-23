#ifndef SET_LOCATION_H
#define SET_LOCATION_H

#include <geometry_msgs/Pose.h>

/* Set Locations to blackboard by name */
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

        config_.blackboard->set<geometry_msgs::Pose>(name, pose);

        return BT::NodeStatus::SUCCESS;
    }

private:
    BT::NodeConfiguration config_;
};

#endif // SET_LOCATION_H