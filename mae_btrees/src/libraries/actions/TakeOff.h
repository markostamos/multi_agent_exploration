#ifndef TAKE_OFF_H
#define TAKE_OFF_H

#include <geometry_msgs/Twist.h>

/**
 * @brief Makes the UAV take off at specified height in meters.
 * @param height Height is given in the xml file as a parameter.
 */
extern RosComm state;
class TakeOff : public BT::StatefulActionNode
{
public:
    TakeOff(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
        cmd_vel_pub_ = state.nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
            cmd_vel_pub_.publish(msg);
            return BT::NodeStatus::RUNNING;
        }
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus onRunning()
    {
        if (taskCompleted())
        {
            geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
            cmd_vel_pub_.publish(msg);
            ROS_WARN_STREAM("TakeOff task completed");
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }
    void onHalted()
    {
        geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
        cmd_vel_pub_.publish(msg);
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
    ros::Publisher cmd_vel_pub_;
};

#endif // TAKE_OFF_H