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
        if (!taskCompleted())
        {
            ROS_WARN_STREAM("Land task started");
            geometry_msgs::Twist msg = twistFromVec({0, 0, -0.4});
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
            ROS_WARN_STREAM("Land task completed");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    inline bool taskCompleted()
    {
        return state.pose.position.z < 0.1;
    }

    void onHalted()
    {
        ROS_WARN_STREAM("Land task halted");
        geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
        state.publishers.commandVel.publish(msg);
    }

private:
    double height_;
};

#endif // LAND_H
