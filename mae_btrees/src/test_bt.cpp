
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <behaviortree_cpp_v3/bt_factory.h>


#include <mae_btrees/utils.h>
#include <mae_btrees/test_bt_actions.h>
#include <mae_btrees/drone_actions.h>

using namespace BT;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh("~");
  std::string xml_filename;

  nh.param<std::string>("file",xml_filename,"test_tree.xml");
  std::string path = ros::package::getPath("mae_btrees") + "/trees/";

  BehaviorTreeFactory factory;

  factory.registerNodeType<ApproachObject>("ApproachObject");
  
  factory.registerNodeType<update_blackboard>("update_blackboard");

  factory.registerNodeType<GoTo>("GoTo");

  factory.registerSimpleAction("print_blackboard_value", print_blackboard_value, PortsList{InputPort<std::string>("input")});

  
  
  
  
  

  auto tree = factory.createTreeFromFile(path + xml_filename);
  
  PublisherZMQ publisher_zmq(tree);

  NodeStatus status = NodeStatus::IDLE;

  while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}
