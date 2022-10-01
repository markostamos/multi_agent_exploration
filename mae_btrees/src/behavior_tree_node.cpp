#include <ros/ros.h>
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <ros/package.h>
#include <mae_btrees/HandleBT.h>
#include <mae_btrees/ros_comm.h>
using namespace BT;

std::string ns;
RosComm state;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ns = nh.getNamespace().c_str();
  HandleBT handler(nh);

  // xml tree file
  std::string xml_filename;
  private_nh.param<std::string>("file", xml_filename, "main_tree.xml");

  std::string path = ros::package::getPath("mae_btrees") + "/trees/" + xml_filename;

  handler.createTree(path);

  NodeStatus status = NodeStatus::IDLE;

  // loggers

  // PublisherZMQ publisher_zmq(handler.tree_);
  //   StdCoutLogger logger_cout(handler.tree_);
  while (ros::ok())
  {
    ros::spinOnce();
    status = handler.tree_.tickRoot();
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }
}
