
#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <ros/ros.h>
      
class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }

  // You must override the virtual function tick()
  BT::NodeStatus tick() override;
};