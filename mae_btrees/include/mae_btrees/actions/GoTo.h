#ifndef GOTO_H
#define GOTO_H

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/LinearMath/Quaternion.h>
#include <mae_btrees/utils.h>

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

    BT::NodeStatus onStart()
    {
        if (!_client.waitForServer(ros::Duration(1.0)))
        {
            ROS_ERROR("Could not connect to action server");
            return BT::NodeStatus::FAILURE;
        }
        std::string goal_string;
        if (!getInput("goal", goal_string))
        {
            throw BT::RuntimeError("Missing input x,y");
        }

        Position2D goal = pos2D_from_string(goal_string);
        _halt_requested.store(false);

        move_base_msgs::MoveBaseGoal msg;
        msg.target_pose.header.frame_id = "world";
        msg.target_pose.header.stamp = ros::Time::now();
        msg.target_pose.pose.position.x = goal.x;
        msg.target_pose.pose.position.y = goal.y;
        msg.target_pose.pose.position.z = 1;

        tf::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, 0);
        msg.target_pose.pose.orientation.x = myQuaternion.x();
        msg.target_pose.pose.orientation.y = myQuaternion.y();
        msg.target_pose.pose.orientation.z = myQuaternion.z();
        msg.target_pose.pose.orientation.w = myQuaternion.w();

        _client.sendGoal(msg);

        return BT::NodeStatus::RUNNING;
    }
    BT::NodeStatus onRunning(){
        if (_halt_requested)
        {
            _client.cancelAllGoals();
            return BT::NodeStatus::FAILURE;
        }

        if (_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else if (_client.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {
            
            return BT::NodeStatus::RUNNING;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    void onHalted(){
        _halt_requested.store(true);
    }

private:
    std::atomic_bool _halt_requested;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient _client;
};
 




#endif // GOTO_H

