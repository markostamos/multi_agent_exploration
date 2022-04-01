#ifndef TEST_BT_ACTIONS_H
#define TEST_BT_ACTIONS_H
#include <behaviortree_cpp_v3/action_node.h>

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  
  

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
};




// This is an asynchronous operation that will run in a separate thread.
// It requires the input port "goal".

#endif // MOVEBASE_BT_NODES_H