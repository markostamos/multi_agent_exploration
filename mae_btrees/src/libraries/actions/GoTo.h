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
          client_(state.ns + "/move_base", true)
    {
        blacklist_pt_pub_ = state.nh->advertise<geometry_msgs::Point>(state.ns + "/blacklist_pt", 1);
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

        if (abs(state.pose.position.x - target_.position.x) < 0.1 &&
            abs(state.pose.position.y - target_.position.y) < 0.1)
        {
            ROS_WARN_STREAM("[GoTo] Agent already at target");
            return BT::NodeStatus::SUCCESS;
        }

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

            if (dist2d(state.pose, target_) < 0.1)
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

    inline float dist2d(const geometry_msgs::Pose &p1, const geometry_msgs::Pose &p2)
    {
        return sqrt(pow(p1.position.x - p2.position.x, 2) + pow(p1.position.y - p2.position.y, 2));
    }

private:
    typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    MoveBaseClient client_;
    geometry_msgs::Pose target_;
    ros::Publisher blacklist_pt_pub_;
};

#endif // GOTO_H
