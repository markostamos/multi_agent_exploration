#include <mae_btrees/utils.h>

template <>
geometry_msgs::Pose BT::convertFromString(StringView str)
{

    auto parts = BT::splitString(str, ',');
    if (parts.size() != 2)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        geometry_msgs::Pose pose;
        pose.position.x = BT::convertFromString<double>(parts[0]);
        pose.position.y = BT::convertFromString<double>(parts[1]);
        pose.position.z = 1;

        // ignore orientation
        tf::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, 0);
        pose.orientation.x = myQuaternion.x();
        pose.orientation.y = myQuaternion.y();
        pose.orientation.z = myQuaternion.z();
        pose.orientation.w = myQuaternion.w();
        return pose;
    }
}

float calcDistPoses(geometry_msgs::Pose pose1, geometry_msgs::Pose pose2)
{
    float x = pose1.position.x - pose2.position.x;
    float y = pose1.position.y - pose2.position.y;
    return sqrt(x * x + y * y);
}