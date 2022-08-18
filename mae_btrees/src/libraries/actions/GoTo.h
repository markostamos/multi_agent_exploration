#ifndef GOTO_H
#define GOTO_H

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
/* Go to a location
    input: blackboard key
    returns: Running until the UAV reaches the specified location
 */
extern RosComm state;
class GoTo : public BT::StatefulActionNode
{
public:
    GoTo(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config),
          config_(config),
          client_(state.ns + "/move_base", true)
    {
        blacklist_pt_pub_ = state.nh->advertise<geometry_msgs::Point>(state.ns + "/blacklist_pt", 1);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("target")};
    }

    BT::NodeStatus onStart()
    {
        if (!client_.waitForServer(ros::Duration(10.0)))
        {
            ROS_ERROR("Failed to connect to move_base action server");
            return BT::NodeStatus::FAILURE;
        }
        std::string goal_string;

        if (getInput<std::string>("target", goal_string))
        {

            goal_ = config_.blackboard->get<geometry_msgs::Pose>(goal_string);

            move_base_msgs::MoveBaseGoal msg;
            msg.target_pose.header.frame_id = "world";
            msg.target_pose.header.stamp = ros::Time::now();
            msg.target_pose.pose = goal_;
            client_.sendGoal(msg);
        }
        else
        {
            ROS_ERROR("No goal specified");
            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        if (state.cancel_goal_req)
        {
            client_.cancelGoal();
            state.cancel_goal_req = false;
            return BT::NodeStatus::FAILURE;
        }

        if (client_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else if (client_.getState() == actionlib::SimpleClientGoalState::ACTIVE)
        {

            return BT::NodeStatus::RUNNING;
        }
        // TODO: if goal is rejected must be removed from frontiers
        else if (client_.getState() == actionlib::SimpleClientGoalState::ABORTED)
        {
            blacklist_pt_pub_.publish(goal_);
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    void onHalted()
    {
        state.cancel_goal_req = true;
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient client_;
    BT::NodeConfiguration config_;
    geometry_msgs::Pose goal_;
    ros::Publisher blacklist_pt_pub_;
};

#endif // GOTO_H
