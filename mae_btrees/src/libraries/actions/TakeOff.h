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
        {
            ROS_ERROR("No height specified");
            return BT::NodeStatus::FAILURE;
        }
        else
        {
            geometry_msgs::Twist msg = twistFromVec({0, 0, 0.2});
            state.publishers.commandVel.publish(msg);
        }

        halt_requested_.store(false);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        if (halt_requested_)
        {
            return BT::NodeStatus::FAILURE;
        }
        if (std::abs(state.pose.position.z - height_) < 0.1)
        {
            geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
            state.publishers.commandVel.publish(msg);
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted()
    {
        halt_requested_.store(true);
    }

private:
    std::atomic_bool halt_requested_;
    double height_;
};

#endif // TAKE_OFF_H