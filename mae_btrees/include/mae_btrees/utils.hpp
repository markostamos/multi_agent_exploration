#ifndef UTILS_H
#define UTILS_H


#include <tf/LinearMath/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

// Template specialization to converts a string to geometry_msgs::Pose
namespace BT
{
    template <>
    inline geometry_msgs::Pose convertFromString(StringView str)
    {
      
        auto parts = splitString(str, ',');
        if (parts.size() != 2)
        {
            throw RuntimeError("invalid input)");
        }
        else
        {
            geometry_msgs::Pose pose;
            pose.position.x = convertFromString<double>(parts[0]);
            pose.position.y = convertFromString<double>(parts[1]);
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
} // end namespace BT


geometry_msgs::Pose poseMsgFromVec(std::vector<double> pos,std::vector<double> rpy = {0,0,0}){
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

geometry_msgs::PoseStamped poseStampedMsgFromVec(std::vector<double> pos,std::vector<double> rpy = {0,0,0}){
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world";
    msg.pose = poseMsgFromVec(pos,rpy);
    return msg;
}

geometry_msgs::Twist twistMsgFromVec(std::vector<double> linear = {0,0,0} ,std::vector<double> angular ={0,0,0}){
    geometry_msgs::Twist msg;
    msg.linear.x = linear[0];
    msg.linear.y = linear[1];
    msg.linear.z = linear[2];
    msg.angular.x = angular[0];
    msg.angular.y = angular[1];
    msg.angular.z = angular[2];
    return msg;
}

#endif // UTILS_H