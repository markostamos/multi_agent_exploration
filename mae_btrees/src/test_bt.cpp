
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/package.h>

#include <mae_btrees/utils.hpp>
#include <mae_btrees/InitBT.hpp>
using namespace BT;

std::string ns;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree_handler");
  ros::NodeHandle nh;
  ns = nh.getNamespace().c_str();
  InitBT handler(nh);

  // xml tree file
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "test_tree.xml");
  std::string path = ros::package::getPath("mae_btrees") + "/trees/" + xml_filename;

  handler.createTree(path);

  NodeStatus status = NodeStatus::IDLE;
  PublisherZMQ publisher_zmq(handler.tree);

  while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = handler.tree.tickRoot();

    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }
}