#ifndef LAND_H
#define LAND_H
#include <geometry_msgs/Twist.h>

extern RosComm state;
class Land : public BT::StatefulActionNode
{
public:
    Land(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus onStart()
    {
        geometry_msgs::Twist msg = twistMsgFromVec({0, 0, -0.2});
        state.publishers.commandVel.publish(msg);

        halt_requested_.store(false);

        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning()
    {
        if (halt_requested_)
        {
            return BT::NodeStatus::FAILURE;
        }
        if (state.pose.position.z < 0.25)
        {
            geometry_msgs::Twist msg = twistMsgFromVec({0, 0, 0});
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

#endif // LAND_H