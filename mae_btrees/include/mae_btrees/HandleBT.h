#ifndef HANDLE_BT_H
#define HANDLE_BT_H
#include <mae_btrees/actions.h>
class HandleBT {

    

public:
    BT::Tree tree; 
    HandleBT(ros::NodeHandle &nh):nh(nh) {
        
        pose_subscriber = nh.subscribe("/drone1/ground_truth/pose", 100, &HandleBT::position_callback,this);
    }

    void create_tree(std::string path)
    {
        BT::BehaviorTreeFactory factory;

        factory.registerNodeType<ApproachObject>("ApproachObject");
        factory.registerNodeType<GoTo>("GoTo");
        factory.registerNodeType<SaySomething>("SaySomething");
        
        tree = factory.createTreeFromFile(path);
    }


    ros::NodeHandle nh;
    ros::Subscriber pose_subscriber;


    void position_callback(const geometry_msgs::Pose::ConstPtr &msg);
    
};






void HandleBT::position_callback(const geometry_msgs::Pose::ConstPtr &msg)
{
    tree.blackboard_stack.back()->set<geometry_msgs::Pose>("pose", *msg);
    
};


#endif // HANDLE_BT_H