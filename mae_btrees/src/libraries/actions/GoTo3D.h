#ifndef GOTO_3D_H
#define GOTO_3D_H

#include <mae_control/MoveBase3DAction.h>
#include <mae_control/MoveBase3DActionGoal.h>
#include <geometry_msgs/Pose.h>

/**
 * @brief Action node that sends a goal to the 3d move_base action server.
 *
 * @returns SUCCESS if the goal was reached, FAILURE otherwise. If a goal is unreachable it will publish it in the blacklist topic.
 *
 */
extern RosComm state;
class GoTo3D : public BT::StatefulActionNode
{
public:
    GoTo3D(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config),
          client_("/move_base_3d", true)
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
            throw BT::RuntimeError("[GoTo3D] Could not connect to move_base_3D server");
        }
        if (!getInput<geometry_msgs::Pose>("Target", target_))
        {
            ROS_WARN_STREAM("[GoTo3D] Could not get target pose, returning SUCCESS");
            return BT::NodeStatus::SUCCESS;
        }

        // Check if the agent is already at the target
        if (dist3D(state.pose, target_) < 0.2)
        {
            ROS_WARN_STREAM("[GoTo3D] Agent already at target");
            return BT::NodeStatus::SUCCESS;
        }

        // Send goal to actionbase
        mae_control::MoveBase3DGoal msg;
        msg.target = target_;
        if (msg.target.position.z == 0)
            msg.target.position.z = state.pose.position.z;
        ROS_WARN_STREAM("[GoTo3D] Sending goal (" << target_.position.x << "," << target_.position.y << "," << target_.position.z << ")");
        client_.sendGoal(msg);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        // TODO: Check if the goal is unreachable and publish it in the blacklist topic
        // TODO: other functionalities

        if (dist3D(state.pose, target_) < 0.2)
        {
            ROS_WARN_STREAM("[GoTo3D] Goal reached");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
        /*  } */
    }

    void onHalted() override
    {
        client_.cancelGoal();
        ROS_WARN_STREAM("[GoTo3D] Halted");
    }

private:
    typedef actionlib::SimpleActionClient<mae_control::MoveBase3DAction> MoveBaseClient;

    MoveBaseClient client_;
    geometry_msgs::Pose target_;
    ros::Publisher blacklist_pt_pub_;
};

#endif // GOTO_3D_H
