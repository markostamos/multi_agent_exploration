#ifndef LAND_H
#define LAND_H
#include <geometry_msgs/Twist.h>

/**
 * @brief Action node that sends cmd_vel commands to the agent until it lands.
 *        Currently assumes that z=0.5 is the ground.
 *        //TODO: get min z from lidar data to make it more robust.
 *
 */
extern RosComm state;
class Land : public BT::StatefulActionNode
{
public:
    Land(const std::string &name, const BT::NodeConfiguration &config)
        : StatefulActionNode(name, config)
    {
        cmd_vel_pub_ = state.nh->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
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
            ROS_WARN_STREAM("Land task completed");
            return BT::NodeStatus::SUCCESS;
        }
        geometry_msgs::Twist msg = twistFromVec({0, 0, -0.4});
        cmd_vel_pub_.publish(msg);
        return BT::NodeStatus::RUNNING;
    }

    inline bool taskCompleted()
    {
        return state.pose.position.z < 0.5;
    }

    void onHalted()
    {
        ROS_WARN_STREAM("Land task halted");
        geometry_msgs::Twist msg = twistFromVec({0, 0, 0});
        cmd_vel_pub_.publish(msg);
    }

private:
    double height_;
    ros::Publisher cmd_vel_pub_;
};

#endif // LAND_H
