#include <mae_btrees/bt_actions.h>

BT::NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    ROS_INFO("WORKED");

    return BT::NodeStatus::SUCCESS;
}
