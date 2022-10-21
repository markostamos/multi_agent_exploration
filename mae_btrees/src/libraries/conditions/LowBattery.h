#ifndef LOW_BATTERY
#define LOW_BATTERY

extern RosComm state;

/**
 * @brief Dummy node to test the reactivity of Behavior Tree on Safety Tasks.
 *
 */
BT::NodeStatus LowBattery()
{
    static auto now = 0;
    static bool should_recharge = false;
    if (state.battery_percentage < 20)
    {
        should_recharge = true;
        return BT::NodeStatus::SUCCESS;
    }
    else if (should_recharge && state.battery_percentage > 90)
    {
        should_recharge = false;
        ROS_INFO_STREAM("BATTERY RECHARGED TO " << state.battery_percentage);
        return BT::NodeStatus::FAILURE;
    }
    else if (should_recharge && state.battery_percentage < 90)
    {
        return BT::NodeStatus::SUCCESS;
    }

    if (ros::Time::now().toSec() - now > 1 && !should_recharge)
    {
        state.battery_percentage -= 5;
        now = ros::Time::now().toSec();
        ROS_INFO_STREAM("Battery percentage: " << state.battery_percentage);
    }

    return BT::NodeStatus::FAILURE;
}

#endif // LOW_BATTERY
