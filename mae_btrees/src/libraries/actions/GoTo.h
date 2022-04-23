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
            geometry_msgs::Pose goal = config_.blackboard->get<geometry_msgs::Pose>(goal_string);
            // send target to actionlib
            move_base_msgs::MoveBaseGoal msg;
            msg.target_pose.header.frame_id = "world";
            msg.target_pose.header.stamp = ros::Time::now();
            msg.target_pose.pose = goal;

            client_.sendGoal(msg);
        }
        else
        {
            ROS_ERROR("No goal specified");
            return BT::NodeStatus::FAILURE;
        }

        halt_requested_.store(false);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        if (halt_requested_)
        {
            client_.cancelAllGoals();
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
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }
    void onHalted()
    {
        halt_requested_.store(true);
    }

private:
    std::atomic_bool halt_requested_;
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient client_;
    BT::NodeConfiguration config_;
};

#endif // GOTO_H
