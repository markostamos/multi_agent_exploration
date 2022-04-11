
#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
#include <ros/package.h>

#include <mae_btrees/utils.h>
#include <mae_btrees/HandleBT.h>
using namespace BT;







int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_tree_handler");
  ros::NodeHandle nh;
  HandleBT handler(nh);

  //xml tree file
  std::string xml_filename;
  nh.param<std::string>("file", xml_filename, "test_tree.xml");
  std::string path = ros::package::getPath("mae_btrees") + "/trees/" + xml_filename;
  
  handler.create_tree(path);



  NodeStatus status = NodeStatus::IDLE;
  PublisherZMQ publisher_zmq(handler.tree);
  while (ros::ok() && (status == NodeStatus::IDLE || status == NodeStatus::RUNNING))
  {
    ros::spinOnce();
    status = handler.tree.tickRoot();
    
    //std::cout << status << std::endl;
    ros::Duration sleep_time(0.01);
    sleep_time.sleep();
  }

}