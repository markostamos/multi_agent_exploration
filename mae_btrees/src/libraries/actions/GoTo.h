#ifndef GOTO_H
#define GOTO_H

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
/**
 * @brief Action node that sends a goal to the 2d move_base action server.
 *
 * @returns SUCCESS if the goal was reached, FAILURE otherwise. If a goal is unreachable it will publish it in the blacklist topic.
 *
 */
extern RosComm state;
class GoTo : public BT::StatefulActionNode
{
public:
    GoTo(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config),
          client_("/move_base", true)
    {
        blacklist_pt_pub_ = state.nh->advertise<geometry_msgs::Point>("/blacklist_pt", 1);
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<geometry_msgs::Pose>("Target")};
    }

    BT::NodeStatus onStart() override
    {
        if (!client_.waitForServer(ros::Duration(10.0)))
        {
            throw BT::RuntimeError("[GoTo] Could not connect to move_base server");
        }
        if (!getInput<geometry_msgs::Pose>("Target", target_))
        {
            ROS_WARN_STREAM("[GoTo] Could not get target pose, returning SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        // Check if the agent is already at the target
        if (dist2D(state.pose, target_) < 0.1)
        {
            ROS_WARN_STREAM("[GoTo] Agent already at target");
            return BT::NodeStatus::SUCCESS;
        }

        // Send goal to actionbase
        move_base_msgs::MoveBaseGoal msg;
        msg.target_pose.header.frame_id = "world";
        msg.target_pose.header.stamp = ros::Time::now();
        msg.target_pose.pose = target_;
        ROS_WARN_STREAM("[GoTo] Sending goal (" << target_.position.x << "," << target_.position.y << "," << target_.position.z << ")");
        client_.sendGoal(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {

        switch (client_.getState().state_)
        {
        case actionlib::SimpleClientGoalState::SUCCEEDED:
            ROS_WARN_STREAM("[GoTo] Goal reached");
            return BT::NodeStatus::SUCCESS;
        case actionlib::SimpleClientGoalState::ABORTED:
            ROS_WARN_STREAM("[GoTo] Goal aborted");
            blacklist_pt_pub_.publish(target_.position);
            return BT::NodeStatus::FAILURE;
        case actionlib::SimpleClientGoalState::REJECTED:
            ROS_WARN_STREAM("[GoTo] Goal rejected");
            return BT::NodeStatus::FAILURE;
        default:

            if (dist2D(state.pose, target_) < 0.1)
            {
                ROS_WARN_STREAM("[GoTo] Goal reached");
                return BT::NodeStatus::SUCCESS;
            }
            return BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        client_.cancelGoal();
        ROS_WARN_STREAM("[GoTo] Halted");
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient client_;
    geometry_msgs::Pose target_;
    ros::Publisher blacklist_pt_pub_;
};

#endif // GOTO_H
