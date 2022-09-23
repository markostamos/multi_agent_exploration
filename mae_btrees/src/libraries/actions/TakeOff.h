#ifndef TAKE_OFF_H
#define TAKE_OFF_H

#include <geometry_msgs/Twist.h>

/*
    Takes Off at a specified height
 */
extern RosComm state;
class TakeOff : public BT::StatefulActionNode
{
public:
    TakeOff(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<double>("Height")};
    }

    BT::NodeStatus onStart()
    {

        if (!getInput<double>("Height", height_))
            throw BT::RuntimeError("missing required input [Height]");

        if (shouldStart())
        {
            ROS_WARN_STREAM("TakeOff task started");
            geometry_msgs::Twist msg = twistFromVec({0, 0, 0.4});
            state.publishers.commandVel.publish(msg);
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning()
    {
        if (taskCompleted())
        {
            geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
            state.publishers.commandVel.publish(msg);
            ROS_WARN_STREAM("TakeOff task completed");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted()
    {
        geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
        state.publishers.commandVel.publish(msg);
        ROS_WARN_STREAM("TakeOff Halted");
    }

    inline bool shouldStart()
    {
        return state.pose.position.z < height_ - 0.1;
    }

    inline bool taskCompleted()
    {
        return state.pose.position.z >= height_ - 0.1;
    }

private:
    double height_;
};

#endif // TAKE_OFF_H