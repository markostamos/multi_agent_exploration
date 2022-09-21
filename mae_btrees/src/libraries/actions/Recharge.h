#ifndef RECHARGE_H
#define RECHARGE_H

/* Set Locations to blackboard by name */
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
