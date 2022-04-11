#ifndef TEST_BT_ACTIONS_H
#define TEST_BT_ACTIONS_H

#include <geometry_msgs/Pose.h>

class ApproachObject : public BT::SyncActionNode
{
public:
  ApproachObject(const std::string &name) : BT::SyncActionNode(name, {})
  {
  }  

  // You must override the virtual function tick()
  BT::NodeStatus tick(){ 
    std::cout << "ApproachObject: " << this->name() << std::endl;
    

    return BT::NodeStatus::SUCCESS;
    }
};

// SyncActionNode (synchronous action) with an input port.
class SaySomething : public BT::SyncActionNode
{
public:
  // If your Node has ports, you must use this constructor signature
  SaySomething(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), config(config)
  {
  }

  // It is mandatory to define this static method.
  static BT::PortsList providedPorts()
  {
    // This action has a single input port called "message"
    // Any port must have a name. The type is optional.
    return {BT::InputPort<double>("test")};
    // return {BT::InputPort<geometry_msgs::Pose>("test")};
  }

  // As usual, you must override the virtual function tick()
  BT::NodeStatus tick() override
  {
    BT::Optional<double> msg = getInput<double>("test");
    // Check if optional is valid. If not, throw its error
    if (!msg)
    {
      throw BT::RuntimeError("missing required input [message]: ",
                             msg.error());
    }

    // use the method value() to extract the valid message.
    // std::cout << "Robot says: " << msg.value() << std::endl;
    geometry_msgs::Pose pose = config.blackboard->get<geometry_msgs::Pose>("pose");

    // std::cout << "test: " << test << std::endl;
    std::cout << "Robot says: " << pose.position.x << std::endl;
    return BT::NodeStatus::SUCCESS;
  }

private:
  BT::NodeConfiguration config;
};

#endif // TEST_BT_ACTIONS_H