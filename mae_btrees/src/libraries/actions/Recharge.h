#ifndef RECHARGE_H
#define RECHARGE_H

/**
 * @brief Dummy action node to test the reactivity of Behavior Tree on Safety Tasks.
 */
extern RosComm state;
BT::NodeStatus Recharge()
{
    static auto now = 0;
    if (ros::Time::now().toSec() - now > 1)
    {
        state.battery_percentage += 10;
        ROS_INFO_STREAM("Battery percentage: " << state.battery_percentage);
        now = ros::Time::now().toSec();
    }
    return BT::NodeStatus::SUCCESS;
}
#endif // RECHARGE_H
