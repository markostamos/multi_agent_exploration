#include <mae_btrees/test_bt_actions.h>
#include <mae_btrees/utils.h>
BT::NodeStatus ApproachObject::tick()
{
    std::cout << "ApproachObject: " << this->name() << std::endl;
    ROS_INFO("WORKED");

    return BT::NodeStatus::SUCCESS;
}
