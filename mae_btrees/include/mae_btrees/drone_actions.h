#ifndef DRONE_ACTIONS_H
#define DRONE_ACTIONS_H

#include <ros/ros.h>
#include <behaviortree_cpp_v3/action_node.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

class GoTo : public BT::StatefulActionNode
{
public:
    // Any TreeNode with ports must have a constructor with this signature
    GoTo(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config),
          _client("/"+name+"/move_base", true)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("goal")};
    }

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    std::atomic_bool _halt_requested;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient _client;
};
 




#endif

