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

geometry_msgs::Pose poseMsgFromVec(std::vector<double> pos, std::vector<double> rpy)
{
    geometry_msgs::Pose msg;
    msg.position.x = pos[0];
    msg.position.y = pos[1];
    msg.position.z = pos[2];
    tf::Quaternion myQuaternion;
    myQuaternion.setRPY(rpy[0], rpy[1], rpy[2]);
    msg.orientation.x = myQuaternion.x();
    msg.orientation.y = myQuaternion.y();
    msg.orientation.z = myQuaternion.z();
    msg.orientation.w = myQuaternion.w();
    return msg;
}

geometry_msgs::PoseStamped poseStampedMsgFromVec(std::vector<double> pos, std::vector<double> rpy)
{
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = poseMsgFromVec(pos, rpy);
    return msg;
}

geometry_msgs::Twist twistMsgFromVec(std::vector<double> linear, std::vector<double> angular)
{
    geometry_msgs::Twist msg;
    msg.linear.x = linear[0];
    msg.linear.y = linear[1];
    msg.linear.z = linear[2];
    msg.angular.x = angular[0];
    msg.angular.y = angular[1];
    msg.angular.z = angular[2];
    return msg;
}