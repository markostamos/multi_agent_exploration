#include <mae_btrees/bt_actions.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/ros.h>


using namespace BT;

//-----------------------------------------------------

  // Simple tree, used to execute once each action.
static const char *xml_text = R"(
 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <ApproachObject name="approach_object"/>
            <ApproachObject name="approach_object2"/>
            <ApproachObject name="approach_object3"/>
            <ApproachObject name="approach_object"/>
            <ApproachObject name="approach_object2"/>
            <ApproachObject name="approach_object3"/>
            <ApproachObject name="approach_object"/>
            <ApproachObject name="approach_object2"/>
            <ApproachObject name="approach_object3"/>
            <ApproachObject name="approach_object"/>
            <ApproachObject name="approach_object2"/>
            <ApproachObject name="approach_object3"/>
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_behavior_tree");
  ros::NodeHandle nh;

  BehaviorTreeFactory factory;

  factory.registerNodeType<ApproachObject>("ApproachObject");
  
  auto tree = factory.createTreeFromText(xml_text);
  PublisherZMQ publisher_zmq(tree);

  NodeStatus status = NodeStatus::IDLE;

  while( ros::ok())
  {
    ros::spinOnce();
    status = tree.tickRoot();
    std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

  return 0;
}
